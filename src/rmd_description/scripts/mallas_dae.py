"""Escritor de mallas COLLADA compartido por step_a_mallas.py y fusion_a_urdf.py."""
import numpy as np


# COLLADA con <up_axis>Z_UP</up_axis>: una STL no dice qué eje es "arriba" y
# Foxglove (Scene -> Mesh up axis, por defecto Y-up) la gira 90°, dejando el robot
# de lado. El .dae lo declara y lo respetan Foxglove y RViz por igual.
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
