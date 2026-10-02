"""STEP (Inventor, Y arriba, mm) -> mallas STL por link en marco ROS (Z arriba, m).

Uso (necesita OpenCascade: pip install cadquery-ocp numpy):
    python scripts/step_a_mallas.py cad/ROBOT_CON_4PATAS.stp meshes/
Imprime en JSON los pivotes, radios y altura al suelo que usa urdf/rmd.urdf.xacro.
"""
import sys, json, struct, math
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool

step, outdir = sys.argv[1], sys.argv[2]
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader(); r.SetNameMode(True); r.ReadFile(step); r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name(l):
    a = TDataStd_Name()
    return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), a) else "?"


def tris(shape):
    BRepMesh_IncrementalMesh(shape, 0.3, False, 0.4, True)
    out = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        f = TopoDS.Face(e.Current()); loc = TopLoc_Location()
        t = BRep_Tool.Triangulation_s(f, loc)
        if t:
            tr = loc.Transformation()
            P = np.array([(lambda p: (p.X(), p.Y(), p.Z()))(t.Node(i).Transformed(tr))
                          for i in range(1, t.NbNodes() + 1)])
            rev = f.Orientation() == TopAbs_REVERSED
            for i in range(1, t.NbTriangles() + 1):
                a, b, c = t.Triangle(i).Get()
                out.append(P[[a - 1, c - 1, b - 1]] if rev else P[[a - 1, b - 1, c - 1]])
        e.Next()
    if not out:
        return np.zeros((0, 3, 3))
    t = np.array(out) * 0.001                       # mm -> m
    return np.stack([t[..., 0], -t[..., 2], t[..., 1]], -1)   # Y arriba -> Z arriba


# Recorre el árbol: (ruta de instancias, nombre de pieza, triángulos en marco ROS)
piezas = []
def walk(label, loc, ruta):
    ref = TDF_Label()
    if st.IsReference_s(label): st.GetReferredShape_s(label, ref)
    else: ref = label
    ruta = ruta + [name(label)]
    if st.IsAssembly_s(ref):
        comps = Sequence_TDF_Label(); st.GetComponents_s(ref, comps)
        for i in range(1, comps.Length() + 1):
            c = comps.Value(i); walk(c, loc * st.GetLocation_s(c), ruta)
    else:
        t = tris(st.GetShape_s(ref).Moved(loc))
        if len(t): piezas.append((ruta, name(ref), t))

roots = Sequence_TDF_Label(); st.GetFreeShapes(roots)
walk(roots.Value(1), TopLoc_Location(), [])


def esquina(t):
    c = t.reshape(-1, 3).mean(0)
    return ('f' if c[0] > 0 else 'r') + ('l' if c[1] > 0 else 'r')

def bbox_centro(t):
    v = t.reshape(-1, 3); return (v.min(0) + v.max(0)) / 2

ESQ = ['fl', 'fr', 'rl', 'rr']
grupos = {'chasis': [], 'motores': []}
for e in ESQ:
    grupos[f'rueda_{e}'] = []; grupos[f'brazo_{e}'] = []; grupos[f'locas_{e}'] = []

for ruta, pieza, t in piezas:
    top = ruta[1] if len(ruta) > 1 else ruta[0]
    if 'chasis' in pieza:
        grupos['chasis'].append(t)
    elif top.startswith('GIM6010'):
        grupos['motores'].append(t)
    elif top.startswith('RUEDA D96'):
        grupos[f'rueda_{esquina(t)}'].append(t)
    elif top.startswith('Conjunto Brazo'):
        grupos[f'brazo_{esquina(t)}'].append(t)
    elif top.startswith('RUEDA LOCA') or top.startswith('DIN 125'):
        grupos[f'locas_{esquina(t)}'].append(t)
    else:
        print('SIN GRUPO:', ruta, pieza, file=sys.stderr)

G = {k: np.concatenate(v) for k, v in grupos.items()}

info = {'ruedas': {}, 'flippers': {}}
for e in ESQ:
    rueda = G[f'rueda_{e}']
    piv = bbox_centro(rueda)                       # eje de la rueda motriz = pivote del flipper
    v = rueda.reshape(-1, 3)
    radio = (v[:, 2].max() - v[:, 2].min()) / 2
    yaw = 0.0 if e[0] == 'f' else math.pi          # igual que ugv_bridge: traseros con yaw pi
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    def a_local(t):
        return (t - piv) @ R                        # R^T (p - piv)
    brazo = a_local(G[f'brazo_{e}']); locas = a_local(G[f'locas_{e}'])
    # Inclinación del brazo en el CAD: punta (centro de las ruedas locas) respecto al pivote.
    punta = bbox_centro(locas)
    incl = math.atan2(punta[2], punta[0])
    # Se endereza para que q=0 sea brazo horizontal (rotación en Y local de +incl).
    c, s = math.cos(incl), math.sin(incl)
    Ry = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])  # aplica rot. Y(+incl) a vectores fila
    G[f'brazo_{e}'] = brazo @ Ry; G[f'locas_{e}'] = locas @ Ry
    G[f'rueda_{e}'] = rueda - piv
    info['ruedas'][e] = {'xyz': piv.round(5).tolist(), 'radio': round(float(radio), 5)}
    info['flippers'][e] = {'xyz': piv.round(5).tolist(), 'yaw': yaw,
                           'inclinacion_cad_deg': round(math.degrees(incl), 2),
                           'largo': round(float(math.hypot(punta[0], punta[2])), 4)}


# COLLADA con <up_axis>Z_UP</up_axis>: una STL no dice qué eje es "arriba" y
# Foxglove (Scene -> Mesh up axis, por defecto Y-up) la gira 90°, dejando el robot
# de lado. El .dae lo declara y lo respetan Foxglove y RViz por igual.
COLORES = {'chasis': (0.70, 0.70, 0.72), 'motores': (0.75, 0.25, 0.20),
           'rueda': (0.15, 0.15, 0.15), 'brazo': (0.29, 0.50, 0.75),
           'locas': (0.85, 0.75, 0.38)}

def write_dae(path, t, rgb):
    v, idx = np.unique(t.reshape(-1, 3).round(7), axis=0, return_inverse=True)
    idx = idx.reshape(-1, 3)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    fmt = lambda a: ' '.join(f'{x:.6g}' for x in a.ravel())
    prim = np.stack([idx, np.repeat(np.arange(len(t))[:, None], 3, 1)], -1)
    c = ' '.join(f'{x:.3f}' for x in rgb)
    with open(path, 'w') as f:
        f.write(f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
 <library_effects><effect id="fx"><profile_COMMON><technique sid="c"><phong>
  <diffuse><color>{c} 1</color></diffuse>
  <specular><color>0.2 0.2 0.2 1</color></specular><shininess><float>20</float></shininess>
 </phong></technique></profile_COMMON></effect></library_effects>
 <library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
 <library_geometries><geometry id="g"><mesh>
  <source id="p"><float_array id="pa" count="{v.size}">{fmt(v)}</float_array>
   <technique_common><accessor source="#pa" count="{len(v)}" stride="3">
    <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
   </accessor></technique_common></source>
  <source id="n"><float_array id="na" count="{n.size}">{fmt(n)}</float_array>
   <technique_common><accessor source="#na" count="{len(n)}" stride="3">
    <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
   </accessor></technique_common></source>
  <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
  <triangles material="m" count="{len(t)}">
   <input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#n" offset="1"/>
   <p>{' '.join(map(str, prim.ravel()))}</p>
  </triangles>
 </mesh></geometry></library_geometries>
 <library_visual_scenes><visual_scene id="s"><node id="nodo">
  <instance_geometry url="#g"><bind_material><technique_common>
   <instance_material symbol="m" target="#mat"/>
  </technique_common></bind_material></instance_geometry>
 </node></visual_scene></library_visual_scenes>
 <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>
""")

for k, t in G.items():
    write_dae(f'{outdir}/{k}.dae', t, COLORES[k.split('_')[0]])

suelo = min(info['ruedas'][e]['xyz'][2] - info['ruedas'][e]['radio'] for e in ESQ)
info['altura_suelo'] = round(-float(suelo), 5)
print(json.dumps(info, indent=1))
