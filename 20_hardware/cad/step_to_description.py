#!/usr/bin/env python3
"""Regenerate guardian_description's visual meshes and measured dimensions
from a STEP assembly.

Run this whenever a new CAD revision replaces the current one — it is the
only place that ever reads numbers out of the STEP file. Nothing in the
xacro should ever be hand-copied from a CAD viewer again; re-run this
script and rebuild instead.

Usage:
    python3 step_to_description.py [path/to/ALL_VN.STEP]

Requires: pip install --break-system-packages cadquery
(cadquery vendors OCP, the OpenCascade Python binding this script uses
directly for assembly-aware STEP reading — cadquery's own high-level API
does not preserve part names/hierarchy on import, so we go one level down.)

What it does:
  1. Opens the STEP with its XCAF/CAF document so part names and each
     instance's real placement in the assembly are available (a plain
     `cadquery.importers.importStep()` collapses everything into one
     nameless shape and loses this).
  2. Buckets every part instance into a group by name (see PART_GROUPS
     below) and unions each group into one compound in the CAD's native
     mm frame.
  3. Picks a robot-frame origin: centered over the wheel footprint in the
     ground plane, at ground level (wheel center height minus wheel
     radius) — matching the existing URDF convention where base_link's
     origin sits on the ground under the robot's centroid.
  4. Applies one rigid transform taking the CAD's Y-up frame to URDF's
     Z-up frame (rotate +90 deg about X) plus the mm -> m unit scale, then
     meshes and exports each group as an STL under
     guardian_description/meshes/.
  5. Writes the measured wheel spacing/radius and lidar mount offsets to
     urdf/dimensions.xacro as <xacro:property> values, included by
     guardian_sim.urdf.xacro instead of hand-typed numbers.
"""
import math
import sys
from pathlib import Path

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

REPO_ROOT = Path(__file__).resolve().parents[2]
DESC_PKG = REPO_ROOT / "30_ros2_ws/src/guardian_description"
MESHES_DIR = DESC_PKG / "meshes"
DIMENSIONS_XACRO = DESC_PKG / "urdf/dimensions.xacro"

DEFAULT_STEP = Path(__file__).parent / "ALL_V1.STEP"

# Which mesh group each named CAD part belongs to. Parts not listed here
# are ignored (fasteners/rods you don't want rendered can just be left
# out). Update this table, not the geometry code, when a new CAD revision
# renames or adds parts.
PART_GROUPS = {
    "wheel": ["wheel"],
    "lidar": ["lidar"],
    "chassis": [
        "20in", "9in", "6in", "3-way", "motor", "motor_fixure", "flange",
        "corner", "battery", "base", "motor_driver_holder", "motor_driver",
        "lidar_mount",
    ],
}


def base_name(cad_name):
    """CAD names come through as e.g. 'wheel_預設' or, for multi-word
    parts, 'Lidar_mount_預設' (the trailing token is the CAD tool's
    auto-generated "default body" placeholder, in the CAD's UI language).
    Strip only that trailing non-ASCII token, not everything after the
    first underscore — a naive first-underscore split collapses
    'Lidar_mount_預設' down to 'lidar', colliding it with the actual
    'lidar_預設' sensor part."""
    parts = cad_name.strip().split("_")
    if len(parts) > 1 and not parts[-1].isascii():
        parts = parts[:-1]
    return "_".join(parts).lower()


def get_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def load_assembly(step_path):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(step_path))
    if str(status) != "IFSelect_ReturnStatus.IFSelect_RetDone":
        raise RuntimeError(f"Failed to read {step_path}: {status}")
    reader.Transfer(doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free_labels = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_labels)
    if free_labels.Length() != 1:
        raise RuntimeError(
            f"Expected exactly one top-level assembly, found "
            f"{free_labels.Length()} — inspect the STEP manually.")
    # `doc` must stay alive as long as `shape_tool` is used — shape_tool
    # doesn't hold its own reference-counted handle to the document, so if
    # `doc` is allowed to go out of scope and get garbage-collected, every
    # later shape_tool call silently returns empty results instead of
    # raising. Returning it here keeps a live reference in the caller.
    return shape_tool, free_labels.Value(1), doc


def iter_instances(shape_tool, root_label):
    """Yields (group, master_name, moved_shape, master_shape,
    rotated_master_shape, translation_mm) for every part instance directly
    under root_label.

    `moved_shape` has the instance's real placement (translation AND
    rotation) in the assembly applied — use this for the one static
    "chassis" compound, where every part's position and orientation
    relative to every other part matters.

    `master_shape` is the same body at its own local origin, completely
    unmoved.

    `rotated_master_shape` is the master body with only the instance's
    ROTATION applied (translation zeroed) — this is what a part reused as
    a single mesh across several links (wheel, lidar) actually needs: it
    must be centered at its own mount/rotation axis (so placement can be
    handled by the URDF joint origin instead), but it must NOT discard how
    that instance is actually oriented relative to its master body. Using
    the plain (unrotated) `master_shape` for that case was the bug behind
    the LIDAR rendering 90 degrees off — the real mounted part is rotated
    relative to its CAD "default" orientation, and that rotation was being
    silently dropped.
    """
    children = TDF_LabelSequence()
    shape_tool.GetComponents_s(root_label, children)
    for i in range(1, children.Length() + 1):
        comp = children.Value(i)
        # GetLocation_s must run before GetReferredShape_s: passing `comp`
        # itself as the out-param below would mutate it in place (the
        # Python binding aliases the label handle), corrupting `comp`.
        loc = shape_tool.GetLocation_s(comp)
        ref = TDF_Label()
        shape_tool.GetReferredShape_s(comp, ref)
        name = get_name(ref) or get_name(comp)
        group = None
        bn = base_name(name)
        for g, names in PART_GROUPS.items():
            if bn in names:
                group = g
                break
        if group is None:
            continue
        master_shape = shape_tool.GetShape_s(ref)
        moved_shape = master_shape.Moved(loc)
        rotation_only = loc.Transformation()
        rotation_only.SetTranslationPart(gp_Vec(0, 0, 0))
        rotated_master_shape = master_shape.Moved(TopLoc_Location(rotation_only))
        tr = loc.Transformation().TranslationPart()
        yield (group, name, moved_shape, master_shape, rotated_master_shape,
               (tr.X(), tr.Y(), tr.Z()))


def bbox_of(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return (xmin, ymin, zmin, xmax, ymax, zmax)


def cad_to_urdf_transform(origin_mm, scale=0.001):
    """Rotate CAD's Y-up frame to URDF's Z-up, forward-facing frame,
    re-centre on `origin_mm` (in CAD mm coordinates), and scale mm -> m.

    Two rotations, always applied together:
      1. +90 deg about X — CAD's Y-up to URDF's Z-up (urdf_z=cad_y).
      2. +180 deg about Z (yaw) — CAD's own +X axis turned out to point at
         this robot's rear, not its front (found by driving the real
         teleop "forward" command in Gazebo and watching the mesh move
         backward — there's no way to detect this from geometry alone,
         since a mecanum vehicle's wheels respond identically to a pure
         forward/backward command regardless of which end is "labeled"
         front; only the body frame's actual facing direction matters).
    Combined: urdf_x=-cad_x, urdf_y=cad_z, urdf_z=cad_y (all relative to
    origin_mm). `cad_point_to_urdf` below MUST apply the exact same
    combined mapping — the two are kept side by side deliberately so a
    future change to one is hard to make without noticing the other.
    """
    ox, oy, oz = origin_mm
    recenter = gp_Trsf()
    recenter.SetTranslationPart(gp_Vec(-ox, -oy, -oz))
    pitch_up = gp_Trsf()
    pitch_up.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.radians(90))
    yaw_flip = gp_Trsf()
    yaw_flip.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(180))
    scale_trsf = gp_Trsf()
    scale_trsf.SetScaleFactor(scale)
    # Apply order: recenter, then pitch_up, then yaw_flip, then scale.
    combined = scale_trsf.Multiplied(yaw_flip.Multiplied(pitch_up.Multiplied(recenter)))
    return combined


def cad_point_to_urdf(x, y, z, origin_mm, scale=0.001):
    """Must stay in lockstep with cad_to_urdf_transform — see its
    docstring. Combined mapping: urdf_x=-cad_x, urdf_y=cad_z, urdf_z=cad_y
    (relative to origin_mm)."""
    ox, oy, oz = origin_mm
    x, y, z = x - ox, y - oy, z - oz
    return (-x * scale, z * scale, y * scale)


def export_group(shapes, trsf, out_path):
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for shape in shapes:
        builder.Add(compound, shape)
    transformed = BRepBuilderAPI_Transform(compound, trsf, True).Shape()
    BRepMesh_IncrementalMesh(transformed, 0.0005)  # 0.5mm linear deflection
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.Write(transformed, str(out_path))


def main():
    step_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_STEP
    print(f"Reading {step_path} ...")
    shape_tool, root_label, _doc = load_assembly(step_path)

    grouped = {"wheel": [], "lidar": [], "chassis": []}
    # First instance's shape, rotated to match how that instance is
    # actually mounted (translation zeroed) — see iter_instances'
    # docstring for why the plain unrotated master shape isn't enough.
    masters = {"wheel": None, "lidar": None}
    wheel_positions = []
    lidar_positions = []
    wheel_bbox = None

    for group, name, moved, master, rotated_master, translation in iter_instances(
            shape_tool, root_label):
        grouped[group].append(moved)
        if group in masters and masters[group] is None:
            masters[group] = rotated_master
        if group == "wheel":
            wheel_positions.append(translation)
            if wheel_bbox is None:
                wheel_bbox = bbox_of(rotated_master)
        elif group == "lidar":
            lidar_positions.append(translation)

    if len(wheel_positions) != 4:
        raise RuntimeError(
            f"Expected exactly 4 wheel instances, found {len(wheel_positions)} "
            "— check PART_GROUPS / the CAD part names.")

    xs = [p[0] for p in wheel_positions]
    ys = [p[1] for p in wheel_positions]
    zs = [p[2] for p in wheel_positions]
    center_x = (min(xs) + max(xs)) / 2
    center_z = (min(zs) + max(zs)) / 2
    wheel_axle_y = ys[0]  # constant across all 4 wheels

    # wheel_bbox is the LOCAL (unmoved-by-instance-placement) bounding box
    # of one wheel body, in CAD mm — its Y/Z extent gives the wheel
    # diameter regardless of which axis the part models it along.
    wxmin, wymin, wzmin, wxmax, wymax, wzmax = wheel_bbox
    wheel_radius_mm = max(wxmax - wxmin, wzmax - wzmin) / 2
    ground_y = wheel_axle_y - wheel_radius_mm

    origin_mm = (center_x, ground_y, center_z)

    wheel_base_length_m = (max(xs) - min(xs)) / 1000
    wheel_base_width_m = (max(zs) - min(zs)) / 1000
    wheel_radius_m = wheel_radius_mm / 1000

    trsf = cad_to_urdf_transform(origin_mm)
    # Single-instance reusable meshes (wheel, lidar) must be centered on
    # their own local axis, not recentered onto the whole-robot origin —
    # placement per-instance is handled by the URDF joint origin instead.
    local_trsf = cad_to_urdf_transform((0.0, 0.0, 0.0))

    print(f"Measured: wheel_base_length={wheel_base_length_m:.6f} m  "
          f"wheel_base_width={wheel_base_width_m:.6f} m  "
          f"wheel_radius={wheel_radius_m:.6f} m")

    export_group(grouped["chassis"], trsf, MESHES_DIR / "chassis.stl")
    export_group([masters["wheel"]], local_trsf, MESHES_DIR / "wheel.stl")
    print(f"Wrote {MESHES_DIR / 'chassis.stl'}, {MESHES_DIR / 'wheel.stl'}")

    lidar_lines = []
    if grouped["lidar"]:
        export_group([masters["lidar"]], local_trsf, MESHES_DIR / "lidar.stl")
        print(f"Wrote {MESHES_DIR / 'lidar.stl'}")
        for idx, (name, pos) in enumerate(zip(
                ["front", "back"], sorted(lidar_positions, key=lambda p: p[0]))):
            ux, uy, uz = cad_point_to_urdf(*pos, origin_mm)
            lidar_lines.append(
                f'  <xacro:property name="lidar_{name}_x" value="{ux:.6f}"/>\n'
                f'  <xacro:property name="lidar_{name}_y" value="{uy:.6f}"/>\n'
                f'  <xacro:property name="lidar_{name}_z" value="{uz:.6f}"/>')

    dimensions_xacro = f"""<?xml version="1.0"?>
<!-- AUTO-GENERATED by 20_hardware/cad/step_to_description.py from
     {step_path.name} — do not hand-edit, re-run the script instead. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="wheel_base_length" value="{wheel_base_length_m:.6f}"/>
  <xacro:property name="wheel_base_width" value="{wheel_base_width_m:.6f}"/>
  <xacro:property name="wheel_radius" value="{wheel_radius_m:.6f}"/>
{chr(10).join(lidar_lines)}
</robot>
"""
    DIMENSIONS_XACRO.parent.mkdir(parents=True, exist_ok=True)
    DIMENSIONS_XACRO.write_text(dimensions_xacro)
    print(f"Wrote {DIMENSIONS_XACRO}")


if __name__ == "__main__":
    main()
