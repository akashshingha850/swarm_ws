"""
Convert rusko_summer.obj → rusko_summer.glb for Gazebo Harmonic

Adaptive decimation strategy:
  - Separate by material so each category gets an independent COLLAPSE ratio
  - QEM (COLLAPSE) only — no PLANAR pass.  Photogrammetry meshes have micro-surface
    variation even on "flat" areas; PLANAR dissolves that into large N-gons which
    triangulate into long thin slivers and produce shading artifacts.
  - After decimation: merge-by-distance → recalculate normals → smooth shading.
    Smooth shading is the single biggest quality improvement — flat shading makes
    every triangle edge visible on a 2 M-face mesh.

Per-material COLLAPSE ratios (fraction of original faces to keep):
  vegetation  0.50  — complex branching, most detail preserved
  buildings   0.28  — edges matter, moderate reduction
  roads/flat  0.10  — large flat areas, aggressive reduction
  default     0.20  — anything unrecognised

The tuning loop scales every ratio by the same global factor until the exported
GLB lands in [TARGET_SIZE_MB, MAX_SIZE_MB].

Constraints:
  - No Draco — Gazebo Harmonic (assimp 5.2) does not support KHR_draco_mesh_compression
  - Plain binary GLB only

Run:
  blender --background --python convert_to_glb.py
"""

import bpy
import math
import os
import time

WORLD_DIR = os.path.dirname(os.path.abspath(__file__))
OBJ_PATH  = os.path.join(WORLD_DIR, "rusko_winter.obj")
GLB_PATH  = os.path.join(WORLD_DIR, "rusko_winter.glb")

# ── Size target ──────────────────────────────────────────────────────────────
TARGET_SIZE_MB = 99.0
MAX_SIZE_MB    = 100.0
MAX_ITERATIONS = 6

# ── Smooth shading ───────────────────────────────────────────────────────────
# Faces whose normals differ by more than this angle get a hard edge; below it,
# normals are smoothly interpolated.  60° is a safe default for mixed urban/nature.
SMOOTH_ANGLE_DEG = 60.0

# ── Per-material COLLAPSE ratios ─────────────────────────────────────────────
# Keys are lowercase substrings matched against object name + material names.
# The tuning loop multiplies all ratios by global_scale each iteration.
MATERIAL_RATIOS = {
    "tree":    0.50,
    "veg":     0.50,
    "plant":   0.50,
    "forest":  0.50,
    "leaf":    0.50,
    "build":   0.28,
    "house":   0.28,
    "roof":    0.28,
    "wall":    0.28,
    "road":    0.10,
    "street":  0.10,
    "ground":  0.08,
    "terrain": 0.08,
    "grass":   0.08,
    "_default": 0.20,
}


def log(msg):
    print(f"[convert_glb] {msg}", flush=True)


def material_ratio(obj, global_scale):
    name  = obj.name.lower()
    mats  = [s.material.name.lower() for s in obj.material_slots if s.material]
    token = name + " " + " ".join(mats)
    for key, ratio in MATERIAL_RATIOS.items():
        if key != "_default" and key in token:
            return min(1.0, ratio * global_scale)
    return min(1.0, MATERIAL_RATIOS["_default"] * global_scale)


def polish(obj):
    """Merge duplicate verts, fix normals, apply smooth shading."""
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')

    # Remove duplicate vertices left by OBJ import / decimation
    bpy.ops.mesh.remove_doubles(threshold=0.0001)

    # Outward-consistent normals (QEM can occasionally flip a normal)
    bpy.ops.mesh.normals_make_consistent(inside=False)

    bpy.ops.object.mode_set(mode='OBJECT')

    # Smooth shading — interpolates normals across faces so triangle edges vanish
    bpy.ops.object.shade_smooth()

    # Auto-smooth: faces separated by > SMOOTH_ANGLE_DEG keep a hard edge
    angle = math.radians(SMOOTH_ANGLE_DEG)
    try:
        # Blender 4.1+
        bpy.ops.object.shade_smooth_by_angle(angle=angle)
    except AttributeError:
        # Blender 3.x
        obj.data.use_auto_smooth      = True
        obj.data.auto_smooth_angle    = angle


def decimate_object(obj, global_scale):
    """QEM-decimate obj in-place. Returns (faces_before, faces_after, ratio)."""
    before = len(obj.data.polygons)
    ratio  = material_ratio(obj, global_scale)

    bpy.context.view_layer.objects.active = obj
    dec = obj.modifiers.new(name="Decimate", type='DECIMATE')
    dec.decimate_type            = 'COLLAPSE'
    dec.ratio                    = ratio
    dec.use_collapse_triangulate = True   # GLB requires pure triangles
    bpy.ops.object.modifier_apply(modifier="Decimate")

    after = len(obj.data.polygons)
    return before, after, ratio


def export_glb():
    bpy.ops.export_scene.gltf(
        filepath=GLB_PATH,
        export_format='GLB',
        export_draco_mesh_compression_enable=False,  # Gazebo Harmonic compatibility
        export_image_format='JPEG',
        export_jpeg_quality=95,      # raised from 90 → better texture fidelity
        export_materials='EXPORT',
        export_texcoords=True,
        export_normals=True,         # export smooth per-vertex normals
        use_selection=False,
    )


def main():
    t0 = time.time()
    log(f"OBJ  → {OBJ_PATH}")
    log(f"GLB  → {GLB_PATH}")
    log(f"Target: {TARGET_SIZE_MB}–{MAX_SIZE_MB} MB")

    # ── 1. Empty scene ───────────────────────────────────────────────────────
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # ── 2. Import OBJ ────────────────────────────────────────────────────────
    log("Importing OBJ … (5-15 min for 4.7 GB)")
    bpy.ops.wm.obj_import(filepath=OBJ_PATH)
    log(f"  import done — {(time.time()-t0)/60:.1f} min elapsed")

    # ── 3. Separate by material ──────────────────────────────────────────────
    meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    log(f"  objects after import: {len(meshes)}")

    bpy.ops.object.select_all(action='DESELECT')
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.mesh.separate(type='MATERIAL')

    mesh_objects = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    log(f"  objects after material-separation: {len(mesh_objects)}")
    for o in mesh_objects:
        mats = [s.material.name for s in o.material_slots if s.material]
        log(f"    {o.name!r:40s}  mats={mats}  faces={len(o.data.polygons):,}")

    # Save clean copies for loop restoration
    saved_meshes  = {o.name: o.data.copy() for o in mesh_objects}
    total_original = sum(len(o.data.polygons) for o in mesh_objects)
    log(f"  total original faces: {total_original:,}")

    # ── 4. Tuning loop ───────────────────────────────────────────────────────
    global_scale = 1.0

    for iteration in range(1, MAX_ITERATIONS + 1):
        log(f"\n── Iteration {iteration}/{MAX_ITERATIONS}  "
            f"(global_scale={global_scale:.4f}) ──")

        # Restore original meshes on iterations 2+
        if iteration > 1:
            for o in mesh_objects:
                old    = o.data
                o.data = saved_meshes[o.name].copy()
                bpy.data.meshes.remove(old)

        total_before = total_after = 0
        for o in mesh_objects:
            bf, af, ratio = decimate_object(o, global_scale)
            polish(o)
            total_before += bf
            total_after  += af
            log(f"  {o.name!r:40s}  ratio={ratio:.4f}  "
                f"{bf:>9,} → {af:>9,} faces")

        log(f"  total: {total_before:,} → {total_after:,} faces  "
            f"({(time.time()-t0)/60:.1f} min elapsed)")

        log("  exporting GLB …")
        export_glb()

        size_mb = os.path.getsize(GLB_PATH) / 1_000_000
        log(f"  GLB size: {size_mb:.2f} MB")

        if TARGET_SIZE_MB <= size_mb <= MAX_SIZE_MB:
            log(f"\nSUCCESS on iteration {iteration}: {size_mb:.2f} MB")
            log(f"  file: {GLB_PATH}")
            return

        mid_target   = (TARGET_SIZE_MB + MAX_SIZE_MB) / 2
        global_scale = global_scale * (mid_target / size_mb)
        log(f"  {'too large' if size_mb > MAX_SIZE_MB else 'too small'} — "
            f"next global_scale: {global_scale:.4f}")

    size_mb = os.path.getsize(GLB_PATH) / 1_000_000
    log(f"\nWARNING: reached {MAX_ITERATIONS} iterations, "
        f"final size {size_mb:.2f} MB")
    log(f"  file: {GLB_PATH}")


main()
