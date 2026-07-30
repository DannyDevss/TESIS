"""
robot_env.py
============

Entorno de simulacion del robot movil, preparado para Aprendizaje por
Refuerzo (RL). Es independiente de la interfaz grafica (no usa Qt), por lo que
puede ejecutarse "headless" para entrenar miles de episodios rapido, y al mismo
tiempo lo reutiliza la GUI (`simu.py`) para VISUALIZAR exactamente el mismo
entorno en el que aprende la IA.

Arquitectura RL
---------------
- `RobotEnv`  -> el ENTORNO. Sigue la API estandar de Gymnasium:
      obs, info               = env.reset()
      obs, reward, term, trunc, info = env.step(accion)
  Si tienes `gymnasium` instalado, `RobotEnv` ES un `gym.Env` valido y se puede
  entrenar directamente con stable-baselines3. Si no, funciona igual de forma
  manual (la API es identica).

- `RobotBrain` -> la POLITICA (el "cerebro"). Decide la accion a partir de la
  observacion. Aqui se enchufa la IA futura:
      * `HeuristicBrain` -> comportamiento por defecto (patrulla evitando muros).
      * `RLBrain`        -> PUNTO DE INTEGRACION del modelo entrenado (stub).

Tarea configurada: EVITAR OBSTACULOS / PATRULLAR.
  El agente recibe recompensa por seguir moviendose sin chocar; colisionar
  termina el episodio con una penalizacion fuerte.

Espacio de acciones: DISCRETO (3)
  0 = girar izquierda, 1 = recto, 2 = girar derecha (siempre avanzando).

Observacion: 5 distancias de telemetro normalizadas [0, 1]
  angulos relativos al frente del robot: [-90, -45, 0, +45, +90] grados.
  El indice 0 (-90) es la distancia IZQUIERDA (dl) y el indice 4 (+90) la
  DERECHA (dr); coinciden con como `simu.py` proyecta los muros en el mapa 3D.
"""

import math
import random
import numpy as np

# --- Gymnasium es OPCIONAL ---
# Con el instalado, RobotEnv es un entorno gym valido (entrenable con SB3).
# Sin el, el entorno funciona igual de forma manual.
try:
    import gymnasium as gym
    from gymnasium import spaces
    _HAS_GYM = True
    _EnvBase = gym.Env
except Exception:
    _HAS_GYM = False
    _EnvBase = object


# --- PARAMETROS DEL MUNDO (unidades en centimetros) ---
ARENA_HALF = 250.0          # semilado de la arena cuadrada -> 500x500 cm
MAX_RANGE = 250.0           # alcance maximo de los telemetros (cm)
ROBOT_RADIUS = 12.0         # radio de colision del robot (cm)
SPEED_CM_S = 25.0           # velocidad de avance (cm/s)
TURN_DEG = 6.0              # giro por paso al elegir izquierda/derecha (grados)
DT = 0.05                   # paso de tiempo (s) -> 20 Hz
MAX_STEPS = 3000            # pasos antes de truncar el episodio
NEAR_WALL = 35.0            # umbral "demasiado cerca de un muro" (cm)

# Angulos de los 5 telemetros, relativos al frente del robot (grados).
SENSOR_ANGLES = (-90.0, -45.0, 0.0, 45.0, 90.0)

# Obstaculos internos: (centro_x, centro_y, ancho, alto) en cm.
OBSTACLES = (
    (110.0, -70.0, 70.0, 70.0),
    (-120.0, 90.0, 90.0, 50.0),
    (-40.0, -150.0, 60.0, 60.0),
)

# --- SUBSISTEMA DE DETECCION DE VICTIMAS (perception) ---
# Estas constantes NO afectan al problema RL (no entran en obs/accion/reward);
# modelan los sensores de busqueda de personas (microfonos + camara IR).
VICTIMS = (
    (190.0, 170.0),
    (-200.0, -170.0),
    (-60.0, 210.0),
)
AUDIO_RANGE = 220.0     # alcance de los microfonos (cm)
CAM_FOV = 60.0          # campo de vision horizontal de la camara IR (grados)
CAM_RANGE = 200.0       # alcance util de la camara IR (cm)
P_VOICE = 0.5           # prob. de que una victima en rango vocalice por ciclo
P_NOISE = 0.15          # prob. de un ruido ambiente (el filtro debe descartarlo)

# --- MUNDO 3D (alturas, cm) ---
WALL_H = 260.0          # altura de los muros de la arena
CEILING_H = 260.0       # techo (edificio cerrado)
# Altura de cada obstaculo de OBSTACLES (escombros bajos, columnas, etc.).
OBSTACLE_HEIGHTS = (70.0, 200.0, 45.0)

# --- LiDAR 3D ---
LIDAR_Z = 35.0          # altura de montaje del LiDAR sobre el robot (cm)
LIDAR_RANGE = 320.0     # alcance maximo del LiDAR (cm)
N_AZIMUTH = 120         # haces por vuelta (resolucion horizontal: 3 grados)
N_RINGS = 16            # canales verticales (como un LiDAR multicapa real)
EL_MIN, EL_MAX = -24.0, 18.0   # FOV vertical (grados): abajo al suelo, arriba al techo

# --- TERRENO IRREGULAR (heightmap del suelo, cm) ---
# Suelo ondulado para que el robot se incline (pitch/roll) al recorrerlo. No
# afecta al problema RL (colisiones y sensores de navegacion son 2D).
TERRAIN_AMP = 16.0      # amplitud de las ondulaciones del suelo (cm)


def terrain_height(x, y):
    """Altura del suelo en (x, y). Acepta escalares o arrays de numpy."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    return (TERRAIN_AMP * np.sin(x / 85.0) * np.cos(y / 100.0)
            + 0.45 * TERRAIN_AMP * np.sin((x + y) / 60.0))


def _rect_segments(cx, cy, w, h):
    """Devuelve los 4 segmentos (a, b) de un rectangulo centrado en (cx, cy)."""
    hw, hh = w / 2.0, h / 2.0
    p1 = (cx - hw, cy - hh)
    p2 = (cx + hw, cy - hh)
    p3 = (cx + hw, cy + hh)
    p4 = (cx - hw, cy + hh)
    return [(p1, p2), (p2, p3), (p3, p4), (p4, p1)]


def _build_segments():
    """Muros de la arena + obstaculos, como lista de segmentos."""
    segs = _rect_segments(0.0, 0.0, ARENA_HALF * 2, ARENA_HALF * 2)
    for (cx, cy, w, h) in OBSTACLES:
        segs.extend(_rect_segments(cx, cy, w, h))
    return segs


def _ray_segment_dist(ox, oy, dx, dy, ax, ay, bx, by):
    """
    Interseccion rayo/segmento. El rayo parte de (ox, oy) en direccion unitaria
    (dx, dy); el segmento va de (ax, ay) a (bx, by). Devuelve la distancia a lo
    largo del rayo si hay impacto (t >= 0 y 0 <= u <= 1), o None.
    """
    rx, ry = bx - ax, by - ay              # vector del segmento
    denom = dx * ry - dy * rx              # cross(d, r)
    if abs(denom) < 1e-9:
        return None                        # paralelos
    aox, aoy = ax - ox, ay - oy
    t = (aox * ry - aoy * rx) / denom      # distancia a lo largo del rayo
    u = (aox * dy - aoy * dx) / denom      # parametro sobre el segmento
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


class RobotEnv(_EnvBase):
    """Entorno de patrulla/evitacion de obstaculos para el robot movil."""

    metadata = {"render_modes": []}

    def __init__(self, seed=None):
        super().__init__()
        self._segments = _build_segments()
        self._rng = random.Random(seed)

        # Estado del robot.
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0          # grados; 0 = avanzar hacia -Y (igual que la GUI)
        self.steps = 0
        self.visited = set()        # celdas recorridas (para metrica de patrulla)

        # Victimas en el mundo (fijas). Parte del subsistema de deteccion, no
        # del problema RL.
        self.victims = [tuple(v) for v in VICTIMS]

        # --- Geometria 3D para el LiDAR: segmentos verticales con altura ---
        # (ax, ay, bx, by, altura). Muros de la arena (altura WALL_H) + las
        # 4 caras de cada obstaculo (con su propia altura).
        self._lidar_segments = []
        for (a, b) in _rect_segments(0.0, 0.0, ARENA_HALF * 2, ARENA_HALF * 2):
            self._lidar_segments.append((a[0], a[1], b[0], b[1], WALL_H))
        for i, (cx, cy, w, h) in enumerate(OBSTACLES):
            alt = OBSTACLE_HEIGHTS[i] if i < len(OBSTACLE_HEIGHTS) else 100.0
            for (a, b) in _rect_segments(cx, cy, w, h):
                self._lidar_segments.append((a[0], a[1], b[0], b[1], alt))

        # --- Direcciones de los haces del LiDAR (precalculadas una vez) ---
        # Azimut completo 360 grados x N_RINGS elevaciones. Como el barrido es
        # de 360, no depende del rumbo del robot.
        az = np.linspace(0.0, 2.0 * np.pi, N_AZIMUTH, endpoint=False)
        el = np.radians(np.linspace(EL_MIN, EL_MAX, N_RINGS))
        AZ, EL = np.meshgrid(az, el)
        ce, se = np.cos(EL).ravel(), np.sin(EL).ravel()
        sa, ca = np.sin(AZ).ravel(), np.cos(AZ).ravel()
        # Misma convencion que el movimiento: adelante = (sin, -cos).
        self._dx = (ce * sa).astype(np.float64)
        self._dy = (-ce * ca).astype(np.float64)
        self._dz = se.astype(np.float64)

        # Definicion de espacios (estandar Gymnasium, si esta disponible).
        self.n_actions = 3
        self.obs_dim = len(SENSOR_ANGLES)
        if _HAS_GYM:
            self.action_space = spaces.Discrete(self.n_actions)
            self.observation_space = spaces.Box(
                low=0.0, high=1.0, shape=(self.obs_dim,), dtype=np.float32
            )

    # ------------------------------------------------------------------ API
    def reset(self, *, seed=None, options=None):
        """Reinicia el episodio. Devuelve (observacion, info) al estilo gym."""
        if seed is not None:
            self._rng.seed(seed)
        # Aparece en una posicion libre aleatoria, mirando en direccion random.
        self.x, self.y = self._spawn_libre()
        self.heading = self._rng.uniform(0.0, 360.0)
        self.steps = 0
        self.visited = set()
        obs, _ = self._leer_sensores()
        return obs, self._info(obs)

    def step(self, action):
        """Aplica una accion discreta y avanza la fisica un paso."""
        action = int(action)
        if action == 0:
            self.heading -= TURN_DEG
        elif action == 2:
            self.heading += TURN_DEG
        self.heading %= 360.0

        # Avanza (misma convencion de la GUI: adelante = (sin h, -cos h)).
        rad = math.radians(self.heading)
        nx = self.x + SPEED_CM_S * DT * math.sin(rad)
        ny = self.y - SPEED_CM_S * DT * math.cos(rad)

        terminated = False
        if self._colision(nx, ny):
            # Choca: no se mueve, termina el episodio.
            terminated = True
        else:
            self.x, self.y = nx, ny

        self.steps += 1
        truncated = self.steps >= MAX_STEPS

        obs, dist_cm = self._leer_sensores()
        reward = self._recompensa(dist_cm, terminated)
        return obs, reward, terminated, truncated, self._info(obs)

    # -------------------------------------------------------------- helpers
    def _spawn_libre(self):
        """Busca una posicion de aparicion sin colision."""
        for _ in range(200):
            x = self._rng.uniform(-ARENA_HALF + 40, ARENA_HALF - 40)
            y = self._rng.uniform(-ARENA_HALF + 40, ARENA_HALF - 40)
            if not self._colision(x, y):
                return x, y
        return 0.0, 0.0

    def _colision(self, x, y):
        """True si el robot (inflado por su radio) choca con muros u obstaculos."""
        r = ROBOT_RADIUS
        if x <= -ARENA_HALF + r or x >= ARENA_HALF - r:
            return True
        if y <= -ARENA_HALF + r or y >= ARENA_HALF - r:
            return True
        for (cx, cy, w, h) in OBSTACLES:
            hw, hh = w / 2.0 + r, h / 2.0 + r
            if (cx - hw) <= x <= (cx + hw) and (cy - hh) <= y <= (cy + hh):
                return True
        return False

    def _leer_sensores(self):
        """Lanza los 5 telemetros y devuelve (obs_normalizada, distancias_cm)."""
        dist_cm = np.empty(len(SENSOR_ANGLES), dtype=np.float32)
        for i, rel in enumerate(SENSOR_ANGLES):
            ang = math.radians(self.heading + rel)
            dx, dy = math.sin(ang), -math.cos(ang)
            mejor = MAX_RANGE
            for (ax, ay), (bx, by) in self._segments:
                d = _ray_segment_dist(self.x, self.y, dx, dy, ax, ay, bx, by)
                if d is not None and d < mejor:
                    mejor = d
            dist_cm[i] = mejor
        obs = (dist_cm / MAX_RANGE).astype(np.float32)
        return obs, dist_cm

    def _recompensa(self, dist_cm, terminated):
        """
        Tarea EVITAR OBSTACULOS / PATRULLAR:
          + sobrevivir y avanzar cada paso,
          + bonus por recorrer celdas nuevas (fomenta patrullar, no girar en sitio),
          - penalizacion por acercarse demasiado a un muro,
          - castigo fuerte al colisionar (fin del episodio).
        """
        if terminated:
            return -10.0
        reward = 0.05  # sigue vivo y avanzando
        cell = (int(self.x // 30.0), int(self.y // 30.0))
        if cell not in self.visited:
            self.visited.add(cell)
            reward += 0.2  # celda nueva -> patrulla
        if float(dist_cm.min()) < NEAR_WALL:
            reward -= 0.2  # demasiado cerca de un muro
        return reward

    def _info(self, obs):
        """Diccionario auxiliar; expone dl/dr/frente en cm para la GUI."""
        return {
            "dl": float(obs[0] * MAX_RANGE),   # distancia izquierda (cm)
            "dr": float(obs[4] * MAX_RANGE),   # distancia derecha (cm)
            "front": float(obs[2] * MAX_RANGE),
            "coverage": len(self.visited),
            "x": self.x, "y": self.y, "heading": self.heading,
        }

    # --------------------------------------------- DETECCION DE VICTIMAS
    def _rel_bearing(self, vx, vy):
        """
        Rumbo de un punto (vx, vy) relativo al frente del robot, en grados
        dentro de (-180, 180]: 0 = justo al frente, + = a la derecha.
        Devuelve (rumbo, distancia_cm).
        """
        dx, dy = vx - self.x, vy - self.y
        theta = math.degrees(math.atan2(dx, -dy))   # misma convencion que heading
        bearing = (theta - self.heading + 180.0) % 360.0 - 180.0
        return bearing, math.hypot(dx, dy)

    def sense_audio(self, rng):
        """
        Microfonos con FILTRO de voz humana. Devuelve un evento o None:
          {"human": True,  "bearing": b, "x": vx, "y": vy, "dist": d}  -> voz
          {"human": False, "bearing": b, "x": None, ...}               -> ruido
        El llamador debe descartar los eventos con human=False (eso ES el filtro).
        """
        candidatas = []
        for (vx, vy) in self.victims:
            b, d = self._rel_bearing(vx, vy)
            if d <= AUDIO_RANGE:
                candidatas.append((d, b, vx, vy))
        if candidatas and rng.random() < P_VOICE:
            candidatas.sort()                  # la victima mas cercana vocaliza
            d, b, vx, vy = candidatas[0]
            return {"human": True, "bearing": b, "x": vx, "y": vy, "dist": d}
        if rng.random() < P_NOISE:
            return {"human": False, "bearing": rng.uniform(-180.0, 180.0),
                    "x": None, "y": None, "dist": None}
        return None

    def sense_thermal(self):
        """
        Camara IR: devuelve la victima visible (dentro del FOV y rango) mas
        centrada, o None. 'u' es su posicion horizontal en el frame [0, 1].
        """
        mejor = None
        for (vx, vy) in self.victims:
            b, d = self._rel_bearing(vx, vy)
            if d <= CAM_RANGE and abs(b) <= CAM_FOV / 2.0:
                if mejor is None or abs(b) < abs(mejor["bearing"]):
                    u = 0.5 + b / CAM_FOV
                    mejor = {"bearing": b, "x": vx, "y": vy, "dist": d,
                             "u": min(1.0, max(0.0, u))}
        return mejor

    # ------------------------------------------------------------ LiDAR 3D
    def lidar_scan(self):
        """
        Barrido completo del LiDAR 3D. Lanza N_AZIMUTH x N_RINGS haces desde la
        posicion del robot y devuelve los puntos de impacto como un array
        (K, 3) en coordenadas del mundo (cm). Vectorizado con numpy.

        Superficies consideradas: muros/obstaculos verticales (con su altura),
        el suelo (z=0) y el techo (z=CEILING_H).
        """
        ox, oy, oz = self.x, self.y, LIDAR_Z
        dx, dy, dz = self._dx, self._dy, self._dz
        best = np.full(dx.shape[0], np.inf)

        with np.errstate(divide="ignore", invalid="ignore"):
            # --- Muros y obstaculos (segmentos verticales extruidos) ---
            for (ax, ay, bx, by, h) in self._lidar_segments:
                rx, ry = bx - ax, by - ay
                denom = dx * ry - dy * rx
                ok = np.abs(denom) > 1e-9
                safe = np.where(ok, denom, 1.0)
                aox, aoy = ax - ox, ay - oy
                t = (aox * ry - aoy * rx) / safe
                u = (aox * dy - aoy * dx) / safe
                z = oz + t * dz
                valid = ok & (t > 0) & (u >= 0.0) & (u <= 1.0) & (z >= 0.0) & (z <= h)
                best = np.minimum(best, np.where(valid, t, np.inf))

            # --- Suelo (plano z=0) ---
            down = dz < -1e-6
            tf = np.where(down, -oz / np.where(down, dz, -1.0), np.inf)
            fx, fy = ox + tf * dx, oy + tf * dy
            vf = down & (tf > 0) & (np.abs(fx) < ARENA_HALF) & (np.abs(fy) < ARENA_HALF)
            best = np.minimum(best, np.where(vf, tf, np.inf))

            # --- Techo (plano z=CEILING_H) ---
            up = dz > 1e-6
            tc = np.where(up, (CEILING_H - oz) / np.where(up, dz, 1.0), np.inf)
            cx, cy = ox + tc * dx, oy + tc * dy
            vc = up & (tc > 0) & (np.abs(cx) < ARENA_HALF) & (np.abs(cy) < ARENA_HALF)
            best = np.minimum(best, np.where(vc, tc, np.inf))

        hit = best < LIDAR_RANGE
        t = best[hit]
        pts = np.empty((t.shape[0], 3), dtype=np.float32)
        pts[:, 0] = ox + t * dx[hit]
        pts[:, 1] = oy + t * dy[hit]
        pts[:, 2] = oz + t * dz[hit]

        # Los impactos en el suelo (z≈0) se elevan a la altura real del terreno
        # irregular, de modo que la nube refleje las ondulaciones del piso.
        piso = pts[:, 2] < 2.0
        if np.any(piso):
            pts[piso, 2] = terrain_height(pts[piso, 0], pts[piso, 1]).astype(np.float32)
        return pts

    # ------------------------------------------------ ACTITUD (PITCH/ROLL/YAW)
    def floor_z(self):
        """Altura del suelo bajo el robot (terreno irregular)."""
        return float(terrain_height(self.x, self.y))

    def attitude(self):
        """Inclinación del robot según la pendiente del terreno bajo él.
        Devuelve (pitch, roll) en grados; el yaw es self.heading.
          - pitch: cuesta arriba/abajo en la dirección de avance.
          - roll:  inclinación lateral (a izquierda/derecha)."""
        e = 6.0
        gx = (float(terrain_height(self.x + e, self.y))
              - float(terrain_height(self.x - e, self.y))) / (2.0 * e)
        gy = (float(terrain_height(self.x, self.y + e))
              - float(terrain_height(self.x, self.y - e))) / (2.0 * e)
        rad = math.radians(self.heading)
        fx, fy = math.sin(rad), -math.cos(rad)   # vector de avance
        rx, ry = math.cos(rad), math.sin(rad)    # vector hacia la derecha
        pitch = -math.degrees(math.atan(gx * fx + gy * fy))
        roll = math.degrees(math.atan(gx * rx + gy * ry))
        return pitch, roll


# ====================================================================== BRAINS
class RobotBrain:
    """
    Interfaz de POLITICA de movimiento. La IA futura debe heredar de aqui e
    implementar `decidir(obs) -> accion_discreta (0|1|2)`.
    """
    def reset(self):
        pass

    def decidir(self, obs):
        raise NotImplementedError


class HeuristicBrain(RobotBrain):
    """
    Politica por defecto (sin aprendizaje): patrulla evitando muros.
    Si tiene un obstaculo al frente, gira hacia el lado mas despejado; de vez en
    cuando hace un giro aleatorio para explorar. Da un comportamiento creible
    en la GUI mientras no exista el modelo entrenado.
    """
    def __init__(self, p_giro_aleatorio=0.02):
        self.p_giro_aleatorio = p_giro_aleatorio

    def decidir(self, obs):
        izq, izq45, frente, der45, der = obs
        # Obstaculo al frente o en diagonal: gira al lado mas abierto.
        if frente < 0.30 or izq45 < 0.20 or der45 < 0.20:
            return 0 if (izq + izq45) > (der + der45) else 2
        # Giro exploratorio ocasional.
        if random.random() < self.p_giro_aleatorio:
            return random.choice([0, 2])
        return 1  # recto


class RLBrain(RobotBrain):
    """
    PUNTO DE INTEGRACION de la IA por refuerzo futura.

    Cuando tengas un modelo entrenado (p.ej. PPO de stable-baselines3), cargalo
    aqui y este cerebro reemplaza a `HeuristicBrain` sin tocar nada mas:

        from stable_baselines3 import PPO
        modelo = PPO.load("modelo_robot.zip")
        brain = RLBrain(model=modelo)

    Mientras no haya modelo, va recto (no-op seguro).
    """
    def __init__(self, model=None, model_path=None):
        self.model = model
        if model is None and model_path is not None:
            self.model = self._cargar(model_path)

    @staticmethod
    def _cargar(model_path):
        try:
            from stable_baselines3 import PPO
            return PPO.load(model_path)
        except Exception as e:
            print(f"[RLBrain] No se pudo cargar el modelo ({e}). Uso fallback.")
            return None

    def decidir(self, obs):
        if self.model is None:
            return 1  # sin modelo: recto
        action, _ = self.model.predict(np.asarray(obs, dtype=np.float32),
                                        deterministic=True)
        return int(action)


# ============================================================ DEMO / ENTRENO
def _demo_headless(pasos=5000):
    """Prueba rapida sin GUI: corre la politica heuristica y reporta metricas."""
    env = RobotEnv(seed=0)
    brain = HeuristicBrain()
    obs, _ = env.reset()
    brain.reset()
    episodios, choques, recompensa_total, vivo = 0, 0, 0.0, 0
    for _ in range(pasos):
        accion = brain.decidir(obs)
        obs, reward, term, trunc, info = env.step(accion)
        recompensa_total += reward
        vivo += 1
        if term or trunc:
            episodios += 1
            if term:
                choques += 1
            obs, _ = env.reset()
            brain.reset()
            vivo = 0
    print(f"Pasos: {pasos} | Episodios: {episodios} | Choques: {choques} | "
          f"Recompensa total: {recompensa_total:.1f} | "
          f"gym disponible: {_HAS_GYM}")


if __name__ == "__main__":
    # Ejecuta:  python robot_env.py        -> prueba la heuristica (headless)
    _demo_headless()
