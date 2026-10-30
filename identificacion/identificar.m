function res = identificar(csv_ajuste, varargin)
%IDENTIFICAR  Ajusta el modelo de la planta y calcula las ganancias PI del driver.
%
%   res = identificar('datos/m1_oruga_chirp_....csv')
%   res = identificar(ajuste, 'validar', prbs_csv, 'bw', 120)
%
%   Gemelo en MATLAB de `identificar.py`: mismo método y mismos números. Se
%   incluye porque la memoria de la tesis se escribe con las figuras de MATLAB,
%   no porque haga falta para trabajar.
%
%   NO NECESITA la System Identification Toolbox. El ajuste se hace con mínimos
%   cuadrados sobre la ecuación integrada (ver más abajo), que es `\` de toda la
%   vida. Si la toolbox está instalada, además se contrasta con `tfest` y se
%   avisa si los dos caminos no coinciden.
%
%   OPCIONES
%     'validar'    CSV de un segundo ensayo, con OTRA señal (chirp para ajustar,
%                  PRBS para validar). Es el único número que demuestra algo.
%     'bw'         anchos de banda a tabular [rad/s]. El primero es el elegido.
%     'horizonte'  horizonte de la simulación por ventanas [s]. Por defecto 2.
%
%   MODELOS (uno por familia, y no son intercambiables)
%     Oruga   (IDs 1-4):  J*w' + b*w + Tc*sgn(w) = u
%     Flipper (IDs 5-8):  J*w' + b*w + k*(x-x0) + Tc*sgn(w) = u
%
%   En el flipper la gravedad actúa como MUELLE: alrededor del ángulo de trabajo
%   x0, el par m*g*l*sin(x) se linealiza en k*(x-x0) con k = m*g*l*cos(x0). Un
%   flipper ajustado como si fuera de primer orden da ganancias que "no acaban de
%   ir", porque el modelo no resuena y la planta sí.
%
%   MÉTODO
%   Derivar la velocidad para tener w' amplifica el ruido justo en la banda alta,
%   que es donde el chirp aporta información. Aquí se INTEGRA la ecuación una vez:
%
%       J*(w-w0) + b*Iw + k*Ix + Tc*Itanh - u_off*t = Iu
%
%   Todos los regresores son integrales de señales medidas, y la integración
%   promedia el ruido en vez de amplificarlo.
%
%   UNIDADES: los CSV vienen en crudo del driver (rev, rev/s, par del driver),
%   a propósito. `vel_gain` opera en esas mismas unidades, así que la ganancia
%   sale sin ningún 2*pi ni la reducción 8:1 (que además sigue sin verificar en
%   hardware). Ver la cabecera de `registrar_ensayo.py`.
%
%   Ver también: registrar_ensayo.py, identificar.py, LEEME.md

    p = inputParser;
    p.addParameter('validar', '');
    p.addParameter('bw', [100 50 150 200]);
    p.addParameter('horizonte', 2.0);
    p.parse(varargin{:});
    opt = p.Results;

    EPS_COULOMB = 0.02;   % rev/s, ancho del suavizado de sgn(w)

    d = cargar(csv_ajuste, EPS_COULOMB);
    if strcmp(d.perfil, 'friccion')
        res = analizar_friccion(d, EPS_COULOMB);
        return
    end

    par = ajustar(d, EPS_COULOMB);
    val = [];
    if ~isempty(opt.validar)
        val = cargar(opt.validar, EPS_COULOMB);
    end

    informe(d, par, val, opt, EPS_COULOMB);
    graficar(d, par, val, opt, EPS_COULOMB);
    res = par;
    res.datos = d;
end

% ===================================================================== %
function d = cargar(ruta, ~)
% Lee el CSV y su JSON de metadatos, se queda con la fase de excitación y
% remuestrea a paso constante.
%
% El remuestreo no es cosmético: las tramas llegan cada ~10 ms pero con jitter, y
% la integración, la FFT y la simulación suponen paso constante. Se usa el
% periodo MEDIANO, robusto a las tramas que se pierden de vez en cuando.
    T = readtable(ruta, 'TextType', 'string');
    meta = struct('perfil', "desconocido", 'familia', "oruga");
    jsonf = replace(ruta, '.csv', '.json');
    if isfile(jsonf)
        meta = jsondecode(fileread(jsonf));
    end

    % Sólo la excitación: el calentamiento, la sonda de inercia, el frenado, el
    % asentamiento del regulador y el tanteo son preparación, no datos. La sonda
    % además mete escalones de par que el modelo no debe ver.
    sel = T.fase == string(meta.perfil);
    if sum(sel) < 50
        error('identificar:pocasMuestras', ...
              '%s: sólo %d muestras de excitación. ¿El ensayo abortó? Mirar "motivo_fin".', ...
              ruta, sum(sel));
    end

    t = T.t_s(sel);  t = t - t(1);
    % La entrada del modelo es el par TOTAL: durante el ensayo el regulador lento
    % también mete par y la planta recibe la suma. Ajustar contra `par_exc` haría
    % que ese par de sostén apareciera como error de modelo y se repartiera entre
    % J y b.
    u = T.par_cmd(sel);
    uex = T.par_exc(sel);
    x = T.pos_rev(sel);
    w = T.vel_rev_s(sel);

    dt = median(diff(t));
    tu = (0:floor(t(end) / dt))' * dt;
    % La entrada es escalonada (ZOH): interpolarla linealmente inventaría rampas
    % que el motor nunca vio.
    idx = max(discretize(tu, [t; inf]), 1);

    d = struct('ruta', ruta, 'meta', meta, 'perfil', char(meta.perfil), ...
               'familia', char(meta.familia), 't', tu, 'dt', dt, 'fs', 1 / dt, ...
               'u', u(idx), 'u_exc', uex(idx), ...
               'x', interp1(t, x, tu), 'w', interp1(t, w, tu));
end

function I = integrar(y, dt)
    I = [0; cumsum((y(2:end) + y(1:end-1)) / 2) * dt];
end

% ===================================================================== %
function p = ajustar(d, eps_c)
    w = d.w; x = d.x; t = d.t; dt = d.dt;
    colJ  = w - w(1);
    colb  = integrar(w, dt);
    colk  = integrar(x - x(1), dt);
    colTc = integrar(tanh(w / eps_c), dt);
    coloff = -t;
    objetivo = integrar(d.u, dt);

    % El Coulomb sólo es identificable si el eje CRUZA el cero de velocidad. Si
    % gira siempre en el mismo sentido —que es lo que se busca en la oruga, para
    % que la fricción sea localmente lineal— tanh(w/eps) vale ~1 todo el rato y
    % su columna es indistinguible de la del par constante: el sistema queda casi
    % singular y el reparto entre ambas, arbitrario.
    cruza = max(w) > 3 * eps_c && min(w) < -3 * eps_c;

    nombres = {'J', 'b'};  A = [colJ colb];
    if strcmp(d.familia, 'flipper')
        nombres{end+1} = 'k';   A = [A colk];
    end
    if cruza
        nombres{end+1} = 'Tc';  A = [A colTc];
    end
    nombres{end+1} = 'u_off';   A = [A coloff];

    theta = A \ objetivo;
    p = struct('J', 0, 'b', 0, 'k', 0, 'Tc', 0, 'u_off', 0);
    for i = 1:numel(nombres)
        p.(nombres{i}) = theta(i);
    end
    p.cruza_cero = cruza;
    p.cond = cond(A);

    % Contraste opcional con la System Identification Toolbox. No sustituye al
    % ajuste: sirve para detectar que algo va mal si los dos caminos discrepan.
    if exist('tfest', 'file') == 2
        try
            orden = 1 + strcmp(d.familia, 'flipper');
            sys = tfest(iddata(d.w - mean(d.w), d.u - mean(d.u), d.dt), orden, orden - 1);
            den = sys.Denominator / sys.Numerator(find(sys.Numerator, 1));
            p.tfest_J = den(1);
            if abs(p.tfest_J - p.J) > 0.35 * abs(p.J)
                warning('identificar:discrepancia', ...
                        ['tfest da J = %.5g y el ajuste integral %.5g (>35 %% de ' ...
                         'diferencia). Mirar la gráfica de respuesta en frecuencia ' ...
                         'antes de fiarse de ninguno de los dos.'], p.tfest_J, p.J);
            end
        catch
            % tfest es quisquilloso con datos cortos o mal escalados; que falle
            % no puede tumbar el ajuste principal, que no lo necesita.
        end
    end
end

% ===================================================================== %
function w = simular(p, u, dt, w0, x0, eps_c)
% Integra el modelo con la entrada u (Heun).
%
% w0 no es un adorno: el ensayo arranca en el punto de operación que dejó el
% regulador (3 rev/s en la oruga), no parado. Simular desde cero contra datos que
% empiezan a 3 rev/s da un error enorme y un "fit" que no dice nada.
    n = numel(u);
    w = zeros(n, 1);  x = zeros(n, 1);
    w(1) = w0;  x(1) = x0;
    acel = @(ui, wi, xi) (ui + p.u_off - p.b * wi - p.k * xi ...
                          - p.Tc * tanh(wi / eps_c)) / p.J;
    for i = 1:n-1
        a1 = acel(u(i), w(i), x(i));
        wp = w(i) + a1 * dt;  xp = x(i) + w(i) * dt;
        a2 = acel(u(i+1), wp, xp);
        w(i+1) = w(i) + (a1 + a2) * dt / 2;
        x(i+1) = x(i) + (w(i) + wp) * dt / 2;
    end
end

function w = simular_ventanas(p, d, horizonte, eps_c)
% Simulación reenganchada al estado medido cada `horizonte` segundos.
%
% La simulación libre de 25 s es dura y algo injusta con estas plantas: con b/J
% del orden de 2 rad/s la oruga es casi un integrador y su ganancia en continua
% es enorme, así que un error minúsculo en el par constante se convierte en una
% deriva grande de velocidad al cabo de medio minuto. El modelo puede describir
% perfectamente la DINÁMICA y aun así sacar un fit pobre por esa deriva.
% Reenganchando cada pocos segundos se mide lo que importa para sintonizar un
% lazo: si el modelo predice la respuesta en horizontes comparables a la
% constante de tiempo de la planta.
    n = max(round(horizonte / d.dt), 10);
    w = zeros(size(d.w));
    for i0 = 1:n:numel(d.w)
        i1 = min(i0 + n - 1, numel(d.w));
        w(i0:i1) = simular(p, d.u(i0:i1), d.dt, d.w(i0), d.x(i0) - d.x(1), eps_c);
    end
end

function f = fit_pct(y, yest)
    f = 100 * (1 - norm(y - yest) / norm(y - mean(y)));
end

% ===================================================================== %
function g = ganancias(J, wc, familia, k)
% De la inercia identificada a los tres números del driver.
%
% El lazo de velocidad del driver es T = vel_gain*(w_ref - w), con w en rev/s y T
% en las unidades de par del driver. La planta, en ESAS MISMAS unidades, es
% 1/(J*s). El lazo abierto queda vel_gain/(J*s) y cruza 0 dB en
%
%     wc = vel_gain / J     =>     vel_gain = J * wc      [wc en rad/s]
%
% Sin 2*pi y sin la reducción, porque J se identificó en unidades del driver.
%
% El integrador se coloca por MARGEN DE FASE, no por receta: el cero del PI en
% wc/4 quita ~14 grados de fase en el cruce, peaje asumible a cambio de matar el
% error estacionario. El manual del GIM6010-8 propone 0,5*ancho_de_banda*vel_gain
% sin decir si el ancho de banda va en Hz o en rad/s, y entre las dos lecturas
% hay un factor 2*pi; se calculan las dos para comparar.
%
% El lazo de posición es proporcional puro y entrega velocidad, así que pos_gain
% está en 1/s y su ancho de banda es aproximadamente su propio valor. Se deja en
% wc/4 para respetar la separación entre lazos en cascada.
    g.wc = wc;
    g.vel_gain = J * wc;
    g.vel_integrator_gain = 0.25 * wc * g.vel_gain;
    g.pos_gain = wc / 4;
    g.vig_manual_rads = 0.5 * wc * g.vel_gain;
    g.vig_manual_hz = 0.5 * (wc / (2 * pi)) * g.vel_gain;
    if strcmp(familia, 'flipper') && k > 0
        g.wn_planta = sqrt(k / J);
        g.caida = k / (g.vel_gain * g.pos_gain);
    end
end

% ===================================================================== %
function informe(d, p, val, opt, eps_c)
    fprintf('%s\n', repmat('=', 1, 72));
    fprintf('ENSAYO   %s\n', d.ruta);
    fprintf('  familia %s, perfil %s, %d muestras a %.0f Hz (%.1f ms)\n', ...
            d.familia, d.perfil, numel(d.t), d.fs, d.dt * 1000);
    if isfield(d.meta, 'simulado') && d.meta.simulado
        fprintf('  *** DATOS SIMULADOS: prueban la cadena, no valen para la tesis ***\n');
    end

    fprintf('\nPARÁMETROS IDENTIFICADOS (unidades del driver: rev, rev/s, par del driver)\n');
    fprintf('  J   = %-12.6g inercia          [par/(rev/s^2)]\n', p.J);
    fprintf('  b   = %-12.6g fricción viscosa [par/(rev/s)]\n', p.b);
    if strcmp(d.familia, 'flipper')
        fprintf('  k   = %-12.6g rigidez de gravedad [par/rev]\n', p.k);
    end
    if p.cruza_cero
        fprintf('  Tc  = %-12.6g fricción de Coulomb [par]\n', p.Tc);
    else
        fprintf('  Tc  = (no identificable: el eje no cruzó el cero de velocidad;\n');
        fprintf('        está absorbido en off. Para medirlo, --perfil friccion)\n');
    end
    fprintf('  off = %-12.6g par constante no modelado\n', p.u_off);
    fprintf('  J en unidades SI del mismo eje: %.6g kg*m^2\n', p.J / (2 * pi));
    fprintf('  polo mecánico b/J = %.2f rad/s (%.2f Hz)\n', p.b / p.J, p.b / p.J / 2 / pi);
    if strcmp(d.familia, 'flipper') && p.k > 0
        wn = sqrt(p.k / p.J);
        fprintf('  resonancia de gravedad wn = %.2f rad/s (%.2f Hz), zeta = %.3f\n', ...
                wn, wn / 2 / pi, p.b / (2 * sqrt(p.k * p.J)));
    end
    fprintf('  cond(A) = %.3g', p.cond);
    if p.cond > 1e4
        fprintf('   <-- ALTO: regresores casi dependientes; desconfiar del reparto\n');
        fprintf('            entre J y b (excitar más alto en frecuencia)');
    end
    fprintf('\n');

    fprintf('\nCALIDAD DEL AJUSTE (fit %%, como `compare`)\n');
    fprintf('  %-28s %8s %20s\n', '', 'libre', sprintf('ventanas de %g s', opt.horizonte));
    fprintf('  %-28s %8.1f %20.1f\n', 'sobre los propios datos', ...
            fit_pct(d.w, simular(p, d.u, d.dt, d.w(1), 0, eps_c)), ...
            fit_pct(d.w, simular_ventanas(p, d, opt.horizonte, eps_c)));
    if ~isempty(val)
        fprintf('  %-28s %8.1f %20.1f\n', ['VALIDACIÓN (' val.perfil ')'], ...
                fit_pct(val.w, simular(p, val.u, val.dt, val.w(1), 0, eps_c)), ...
                fit_pct(val.w, simular_ventanas(p, val, opt.horizonte, eps_c)));
        fprintf('  El número que vale es el de VALIDACIÓN por ventanas: señal distinta\n');
        fprintf('  a la del ajuste y sin la deriva que castiga a la simulación libre.\n');
        fprintf('  Por debajo del ~70 %% desconfiar.\n');
    else
        fprintf('  VALIDACIÓN: no se pidió. El fit sobre los propios datos siempre sale\n');
        fprintf('  bien; correr otro ensayo con --perfil prbs y pasarlo en ''validar''.\n');
    end

    fprintf('\nGANANCIAS PROPUESTAS\n');
    fprintf('  (el muestreo de %.0f Hz deja usar hasta ~%.0f rad/s con holgura)\n', ...
            d.fs, 2 * pi * d.fs / 10);
    fprintf('  %11s %7s %12s %21s %10s\n', 'wc [rad/s]', '[Hz]', 'vel_gain', ...
            'vel_integrator_gain', 'pos_gain');
    for wc = opt.bw
        g = ganancias(p.J, wc, d.familia, p.k);
        marca = '';
        if wc == opt.bw(1), marca = '  <-- elegido'; end
        fprintf('  %11.0f %7.1f %12.5g %21.5g %10.4g%s\n', wc, wc / 2 / pi, ...
                g.vel_gain, g.vel_integrator_gain, g.pos_gain, marca);
    end

    g = ganancias(p.J, opt.bw(1), d.familia, p.k);
    fprintf('\n  Para wc = %.0f rad/s, la fórmula del manual daría\n', opt.bw(1));
    fprintf('    vel_integrator_gain = %.5g  (su "ancho de banda" en rad/s: agresivo)\n', ...
            g.vig_manual_rads);
    fprintf('    vel_integrator_gain = %.5g  (leyéndolo en Hz: conservador)\n', ...
            g.vig_manual_hz);
    fprintf('    aquí se propone      %.5g  (cero del PI en wc/4)\n', g.vel_integrator_gain);

    fprintf('\n  Comandos para el driver:\n');
    fprintf('    odrv0.axis0.controller.config.vel_gain = %.5g\n', g.vel_gain);
    fprintf('    odrv0.axis0.controller.config.vel_integrator_gain = %.5g\n', ...
            g.vel_integrator_gain);
    if strcmp(d.familia, 'flipper')
        fprintf('    odrv0.axis0.controller.config.pos_gain = %.5g\n', g.pos_gain);
        if p.k > 0
            fprintf(['\n  Caída del flipper bajo su propio peso con el lazo P de ' ...
                     'posición:\n    ~%.4f rev por rev de desviación (%.1f %%). Si ' ...
                     'molesta, lo arregla\n    el integrador de velocidad, no subir ' ...
                     'pos_gain.\n'], g.caida, g.caida * 100);
        end
    else
        fprintf('    %% pos_gain no se toca: las orugas van en velocidad\n');
    end
    fprintf('\n  Son el PUNTO DE PARTIDA medido, no el final: verificar en el motor con\n');
    fprintf('  un escalón de velocidad. La carga de la oruga cambia con el terreno y la\n');
    fprintf('  del flipper con el ángulo.\n');
    fprintf('%s\n', repmat('=', 1, 72));
end

% ===================================================================== %
function graficar(d, p, val, opt, eps_c)
    west = simular_ventanas(p, d, opt.horizonte, eps_c);
    figure('Name', d.ruta, 'Position', [100 100 900 900]);

    subplot(3, 1, 1);
    plot(d.t, d.u, 'LineWidth', 0.8); hold on;
    plot(d.t, d.u_exc, 'LineWidth', 0.6);
    legend('par total (entrada del modelo)', 'sólo la excitación', 'Location', 'best');
    ylabel('par'); grid on; title(d.ruta, 'Interpreter', 'none');

    subplot(3, 1, 2);
    plot(d.t, d.w, 'LineWidth', 0.8); hold on;
    plot(d.t, west, 'LineWidth', 0.8);
    legend('medido', sprintf('modelo (fit %.1f %%)', fit_pct(d.w, west)), ...
           'Location', 'best');
    xlabel('t [s]'); ylabel('velocidad [rev/s]'); grid on;
    if ~isempty(val)
        wv = simular_ventanas(p, val, opt.horizonte, eps_c);
        title(sprintf('validación cruzada: fit %.1f %% sobre %s', ...
                      fit_pct(val.w, wv), val.perfil));
    end

    % Respuesta en frecuencia empírica (estimador H1) contra el modelo. Es la
    % comprobación que descubre los desajustes que el fit temporal esconde: si el
    % modelo sólo falla en la banda alta, en el tiempo casi no se nota.
    subplot(3, 1, 3);
    nfft = min(1024, floor(numel(d.t) / 4));
    [Puu, f] = pwelch(d.u_exc, hann(nfft), nfft/2, nfft, d.fs);
    Pyu = cpsd(d.u_exc, d.w, hann(nfft), nfft/2, nfft, d.fs);
    ok = Puu > max(Puu) * 1e-3 & f > 0;
    s = 2i * pi * f(ok);
    if strcmp(d.familia, 'flipper')
        Hm = abs(s ./ (p.J * s.^2 + p.b * s + p.k));
    else
        Hm = abs(1 ./ (p.J * s + p.b));
    end
    loglog(f(ok), abs(Pyu(ok) ./ Puu(ok)), 'LineWidth', 0.9); hold on;
    loglog(f(ok), Hm, 'LineWidth', 0.9);
    legend('medido (H1)', 'modelo', 'Location', 'best');
    xlabel('f [Hz]'); ylabel('|w/u|'); grid on;
end

% ===================================================================== %
function r = analizar_friccion(d, eps_c)
% De cada peldaño de par, la velocidad en régimen -> recta b*w + Tc.
%
% Se descarta el primer 60 % de cada peldaño (transitorio) y los peldaños que aún
% derivan más de un 2 % de su velocidad por segundo: un peldaño a medio asentar
% mide una velocidad más baja de la que le toca, y eso INFLA b y hunde Tc.
    u = d.u; w = d.w; t = d.t;
    bordes = [1; find(abs(diff(u)) > 1e-9) + 1; numel(u) + 1];
    pares = []; vels = []; descartados = 0;
    for j = 1:numel(bordes)-1
        i0 = bordes(j); i1 = bordes(j+1) - 1;
        if i1 - i0 < 20, continue; end
        ini = i0 + floor(0.6 * (i1 - i0));
        wss = mean(w(ini:i1));
        c = polyfit(t(ini:i1), w(ini:i1), 1);
        if abs(c(1)) > 0.02 * max(abs(wss), 0.05)
            descartados = descartados + 1;
            continue
        end
        pares(end+1, 1) = u(i0);  %#ok<AGROW>
        vels(end+1, 1) = wss;     %#ok<AGROW>
    end

    moviendo = abs(vels) > 3 * eps_c;
    r = struct('pares', pares, 'vels', vels, 'descartados', descartados);
    fprintf('%s\n', repmat('=', 1, 72));
    fprintf('FRICCIÓN  %s\n', d.ruta);
    fprintf('  %d peldaños en régimen, %d descartados por no asentarse\n', ...
            numel(pares), descartados);
    if sum(moviendo) < 3
        fprintf(['  AVISO: sólo %d peldaños en régimen y en movimiento. Subir ' ...
                 '--t-paso, o\n  --amplitud si es que no se movía.\n'], sum(moviendo));
        r.b = NaN; r.Tc = NaN;
        return
    end
    coef = [vels(moviendo) sign(vels(moviendo))] \ pares(moviendo);
    r.b = coef(1); r.Tc = coef(2);
    fprintf('\n  b  = %.6g   fricción viscosa [par/(rev/s)]\n', r.b);
    fprintf('  Tc = %.6g   fricción de Coulomb [par]\n', r.Tc);
    fprintf(['\n  Par mínimo para arrancar el eje: ~%.4g. Por debajo, el motor no se\n' ...
             '  mueve, y ninguna ganancia arregla eso: es el suelo de la zona muerta\n' ...
             '  del reductor 8:1.\n'], abs(r.Tc));
    fprintf('%s\n', repmat('=', 1, 72));

    figure('Name', ['fricción ' d.ruta]);
    plot(vels, pares, 'o'); hold on;
    ww = linspace(min(vels), max(vels), 200);
    plot(ww, r.b * ww + r.Tc * sign(ww), '-');
    xlabel('velocidad en régimen [rev/s]'); ylabel('par aplicado');
    legend('peldaños medidos', sprintf('b*w + Tc*sgn(w): b=%.4g, Tc=%.4g', r.b, r.Tc), ...
           'Location', 'best');
    grid on; title('Curva de fricción');
end
