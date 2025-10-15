import os, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_FILE = os.path.join(HERE, "l1_base_vs08_mm.dae")  # change if needed

m = trimesh.load(MESH_FILE, force='mesh')

print("Exists:", os.path.exists(MESH_FILE))
print("File size (MB):", os.path.getsize(MESH_FILE)/1e6)
print("Loaded ok?:", m.is_volume if hasattr(m,'is_volume') else not m.is_empty)
print("Faces:", len(m.faces) if hasattr(m, 'faces') else 'n/a')
print("Vertices:", len(m.vertices) if hasattr(m, 'vertices') else 'n/a')
print("Bounds (min,max):", getattr(m, 'bounds', None))
print("Extents (XYZ):", getattr(m, 'extents', None))
print("Centroid:", getattr(m, 'centroid', None))
