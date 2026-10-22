"""Export de Fusion 360 (acdc4robot) -> urdf/rescate.urdf.xacro con los nombres del proyecto.

Uso (solo numpy):
    python3 scripts/fusion_a_urdf.py cad/rescate_export-v1 urdf/rescate.urdf.xacro meshes/rescate

El export trae juntas "Revolucion-N" y links "Eje_LR-v2_3"; aquí se renombran a
la convención de ugv_bridge para que /joint_states de flipper_node mueva el
modelo sin traducir nada:
    flipper_{fl,fr,rl,rr}  junta del eje del flipper (base_link -> flipper_*_link)
    track_{fl,fr,rl,rr}    rueda motriz; se cuelga de base_link y no del flipper,
                           porque su motor gira respecto al chasis
    ruedas locas           fijas: nadie publica su ángulo y sin él
                           robot_state_publisher no emite su TF
La esquina de cada flipper se deduce del signo x/y de su junta, así que un
re-export de Fusion con otra numeración sigue funcionando.
"""
import math
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

export_dir, salida, dir_mallas = sys.argv[1], sys.argv[2], sys.argv[3]
urdf_in = next(f for f in os.listdir(export_dir) if f.endswith('.urdf'))
robot = ET.parse(os.path.join(export_dir, urdf_in)).getroot()


def rpy_a_matriz(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matriz_a_rpy(R):
    p = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    if abs(R[2, 0]) < 1 - 1e-9:
        return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])
    return 0.0, p, math.atan2(-R[0, 1], R[1, 1])


def tf(origin):
    M = np.eye(4)
    if origin is not None:
        M[:3, :3] = rpy_a_matriz(*map(float, origin.get('rpy', '0 0 0').split()))
        M[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return M


def origin(M):
    limpio = lambda v: 0.0 if abs(v) < 1e-9 else round(v, 6)
    xyz = ' '.join(str(limpio(v)) for v in M[:3, 3])
    rpy = ' '.join(str(limpio(v)) for v in matriz_a_rpy(M[:3, :3]))
    return ET.Element('origin', xyz=xyz, rpy=rpy)


juntas = {j.find('child').get('link'): j for j in robot.findall('joint')}
links = {l.get('name'): l for l in robot.findall('link')}
raiz = next(n for n in links if n not in juntas)


def pose(link):
    """Pose del link respecto a la raíz con todas las juntas en 0."""
    if link not in juntas:
        return np.eye(4)
    j = juntas[link]
    return pose(j.find('parent').get('link')) @ tf(j.find('origin'))


def vertices(link):
    v = links[link].find('visual')
    b = open(os.path.join(export_dir, v.find('geometry/mesh').get('filename')), 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    tri = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype(
        [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    pts = tri['v'].reshape(-1, 3).astype(float) * 0.001
    M = pose(link) @ tf(v.find('origin'))
    return pts @ M[:3, :3].T + M[:3, 3]


altura_suelo = -min(vertices(l)[:, 2].min() for l in links)

# --- Clasificar cada link por su papel y su esquina ------------------------- #
nombres, papel = {raiz: 'base_link'}, {}
for link, j in juntas.items():
    if j.find('parent').get('link') != raiz:
        continue
    x, y = pose(link)[:2, 3]
    esquina = ('f' if x > 0 else 'r') + ('l' if y > 0 else 'r')
    nombres[link], papel[link] = f'flipper_{esquina}_link', ('eje', esquina)
    brazo = next(c for c, jj in juntas.items() if jj.find('parent').get('link') == link)
    nombres[brazo], papel[brazo] = f'flipper_{esquina}_brazo', ('brazo', esquina)
    ruedas = [c for c, jj in juntas.items() if jj.find('parent').get('link') == brazo]
    # La motriz es la coaxial con el eje del flipper; las demás, locas.
    ruedas.sort(key=lambda c: np.linalg.norm(pose(c)[[0, 2], 3] - pose(link)[[0, 2], 3]))
    nombres[ruedas[0]], papel[ruedas[0]] = f'track_{esquina}_link', ('motriz', esquina)
    for i, c in enumerate(ruedas[1:], 1):
        nombres[c], papel[c] = f'rueda_loca_{esquina}_{i}', ('loca', esquina)

if len(nombres) != len(links):
    sys.exit(f'links sin clasificar: {sorted(set(links) - set(nombres))}')

# --- Construir el URDF de salida -------------------------------------------- #
XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)
out = ET.Element('robot', name='rescate')
ET.SubElement(out, f'{{{XACRO}}}property', name='mesh_dir',
              value='package://rmd_description/meshes/rescate')
colores = {'base': '0.70 0.70 0.72 1.0', 'eje': '0.75 0.25 0.20 1.0',
           'brazo': '0.29 0.50 0.75 1.0', 'motriz': '0.15 0.15 0.15 1.0',
           'loca': '0.85 0.75 0.38 1.0'}
for nombre, rgba in colores.items():
    ET.SubElement(ET.SubElement(out, 'material', name=nombre), 'color', rgba=rgba)

# base_link es la raíz porque el EKF y la odometría publican odom -> base_link:
# con base_footprint encima tendría dos padres y el TF se rompe. base_footprint
# cuelga debajo, a ras de suelo. La inercia del chasis va en un hijo porque KDL
# no la admite en el link raíz (mismo truco que ugv.urdf.xacro).
ET.SubElement(out, 'link', name='base_footprint')
j = ET.SubElement(out, 'joint', name='base_footprint_joint', type='fixed')
ET.SubElement(j, 'parent', link='base_link')
ET.SubElement(j, 'child', link='base_footprint')
ET.SubElement(j, 'origin', xyz=f'0 0 {-round(altura_suelo, 6)}', rpy='0 0 0')
inercia_chasis = ET.SubElement(out, 'link', name='chassis_inertial_link')
inercia_chasis.append(links[raiz].find('inertial'))
j = ET.SubElement(out, 'joint', name='chassis_inertial_joint', type='fixed')
ET.SubElement(j, 'parent', link='base_link')
ET.SubElement(j, 'child', link='chassis_inertial_link')
ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0')
# imu_link alineado con base_link, igual que en ugv.urdf.xacro (la rotación de
# montaje la aplica el driver): flipper_node publica la IMU en ese marco.
ET.SubElement(out, 'link', name='imu_link')
j = ET.SubElement(out, 'joint', name='imu_joint', type='fixed')
ET.SubElement(j, 'parent', link='base_link')
ET.SubElement(j, 'child', link='imu_link')
ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0')

os.makedirs(dir_mallas, exist_ok=True)
for viejo in sorted(links, key=lambda n: nombres[n]):
    l = links[viejo]
    nuevo = ET.SubElement(out, 'link', name=nombres[viejo])
    if viejo != raiz:
        nuevo.append(l.find('inertial'))
    for tag in ('visual', 'collision'):
        e = l.find(tag)
        malla = e.find('geometry/mesh')
        archivo = os.path.basename(malla.get('filename'))
        shutil.copy(os.path.join(export_dir, malla.get('filename')),
                    os.path.join(dir_mallas, archivo))
        malla.set('filename', '${mesh_dir}/' + archivo)
        if tag == 'visual':
            ET.SubElement(e, 'material', name=papel.get(viejo, ('base',))[0])
        nuevo.append(e)

for hijo, j in juntas.items():
    rol, esquina = papel[hijo]
    padre = j.find('parent').get('link')
    if rol == 'eje':
        nombre, tipo, M = f'flipper_{esquina}', 'continuous', tf(j.find('origin'))
        limite = dict(effort='30.0', velocity='3.0')
    elif rol == 'motriz':
        nombre, tipo, M = f'track_{esquina}', 'continuous', pose(hijo)
        padre, limite = raiz, dict(effort='30.0', velocity='30.0')
    else:
        nombre, tipo, M = f'{nombres[hijo]}_joint', 'fixed', tf(j.find('origin'))
    nueva = ET.SubElement(out, 'joint', name=nombre, type=tipo)
    ET.SubElement(nueva, 'parent', link=nombres[padre])
    ET.SubElement(nueva, 'child', link=nombres[hijo])
    nueva.append(origin(M))
    if tipo == 'continuous':
        eje = np.array(list(map(float, j.find('axis').get('xyz').split())))
        # Convención del proyecto: orugas giran en +Y del chasis (q>0 = avanzar).
        if rol == 'motriz' and (pose(hijo)[:3, :3] @ eje)[1] < 0:
            eje = -eje
        ET.SubElement(nueva, 'axis', xyz=' '.join(str(round(v, 6) + 0.0) for v in eje))
        ET.SubElement(nueva, 'limit', **limite)

ET.indent(out, space='  ')
cabecera = (f'<?xml version="1.0"?>\n<!-- GENERADO por scripts/fusion_a_urdf.py desde '
            f'cad/{os.path.basename(os.path.normpath(export_dir))}. No editar a mano. -->\n')
with open(salida, 'w') as f:
    f.write(cabecera + ET.tostring(out, encoding='unicode') + '\n')

print(f'altura_suelo={altura_suelo:.4f} m')
for viejo in sorted(links, key=lambda n: nombres[n]):
    print(f'  {viejo:16s} -> {nombres[viejo]}')
