# Headless look at a world .glb: import it, print each layer's counts and
# extents, and render one view with Blender's Workbench engine.
#
#   blender -b --python scripts/world-render.py -- WORLD.glb OUT.png EYE TARGET [LENS]
#
# EYE and TARGET are `x,y,z` in the world's local metres (x east, y north,
# z up). Keep the eye above the ground: a camera inside the hill renders a
# grey field and says nothing. Road lines are edge-only meshes, which
# Blender does not render, so they are converted to a bevelled curve first.
import bpy, sys
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
glb, png = argv[0], argv[1]
eye = tuple(float(v) for v in argv[2].split(','))
target = tuple(float(v) for v in argv[3].split(','))
lens = float(argv[4]) if len(argv) > 4 else 35.0

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=glb)
for o in bpy.data.objects:
    if o.type == 'MESH':
        vs = [v.co for v in o.data.vertices]
        zs = [v.z for v in vs]
        print('OBJ', o.name, 'verts', len(vs), 'faces', len(o.data.polygons), 'edges', len(o.data.edges),
              'z %.1f..%.1f' % (min(zs), max(zs)), 'x %.0f..%.0f y %.0f..%.0f' % (
              min(v.x for v in vs), max(v.x for v in vs), min(v.y for v in vs), max(v.y for v in vs)))

terrain = bpy.data.objects['terrain']
terrain.color = (0.55, 0.65, 0.45, 1)
roads = bpy.data.objects.get('roads')
if roads:
    bpy.ops.object.select_all(action='DESELECT')
    roads.select_set(True)
    bpy.context.view_layer.objects.active = roads
    bpy.ops.object.convert(target='CURVE')
    roads.data.bevel_depth = 1.2
    roads.color = (0.9, 0.05, 0.05, 1)

scene = bpy.context.scene
scene.render.engine = 'BLENDER_WORKBENCH'
sh = scene.display.shading
sh.light = 'STUDIO'
sh.color_type = 'OBJECT'
sh.show_shadows = True
sh.show_cavity = True
scene.render.resolution_x = 1400
scene.render.resolution_y = 900
cam_data = bpy.data.cameras.new('cam')
cam_data.lens = lens
cam_data.clip_end = 20000
cam = bpy.data.objects.new('cam', cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
cam.location = Vector(eye)
cam.rotation_euler = (Vector(target) - cam.location).to_track_quat('-Z', 'Y').to_euler()
scene.render.filepath = png
bpy.ops.render.render(write_still=True)
print('RENDERED', png)
