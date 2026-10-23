#!/usr/bin/env python3
"""Work out each link's mass, centre of mass and inertia from the CAD and write urdf/hexapod.inertials.xacro.

The STEP holds the body and one leg (the middle left leg, leg5). Every solid is put in the link that its
top-level component moves with, and given a mass from the table below:

    bought parts   their real weight, spread over the part's CAD volume
    printed parts  the filament's density times PRINT_FILL, unless MEASURED has the printed part's weight

The leg's link frames come from the joint caps: the leg mount's cap is on the coxa axis, the coxa's two
caps on the femur axis and the tibia's two caps on the knee axis. The tibia's x axis runs from the knee
to the centre of the foot ball. The right legs are mirror images of the left (y -> -y). The body gets a
leg mount at each of the six coxa joints in hexapod.urdf.xacro.

Needs the OpenCascade bindings (pip install cadquery-ocp numpy).

Usage: mass_properties.py STEP_FILE [--output urdf/hexapod.inertials.xacro]
"""
import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_ChildIterator, TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

PACKAGE_DIR = Path(__file__).resolve().parent.parent

# CAD frame (mm): +X right, +Y forward, +Z up, origin on the body's underside.
# base_link (m): +x forward, +y left, +z up, on the hip plane 33.85 mm above the underside.
CAD_TO_BASE = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
HIP_HEIGHT = 33.85  # mm

# Coxa joints as in hexapod.urdf.xacro: (x, y, yaw) in mm, on the slimmer body
MIDDLE_Y = 118.36
CORNER_X, CORNER_Y = 119.16, 89.52
LEGS = [(CORNER_X, -CORNER_Y, -math.pi / 4), (0.0, -MIDDLE_Y, -math.pi / 2), (-CORNER_X, -CORNER_Y, -3 * math.pi / 4),
        (-CORNER_X, CORNER_Y, 3 * math.pi / 4), (0.0, MIDDLE_Y, math.pi / 2), (CORNER_X, CORNER_Y, math.pi / 4)]
CAD_LEG_YAW = math.pi / 2  # the STEP's leg is leg5, pointing along +y

# Which link each top-level CAD component moves with
LINKS = {'Body': 'base', 'Body_lid': 'base', 'Camera_pod': 'base', 'Leg_mount': 'base',
         'Coxa': 'coxa', 'Femur': 'femur', 'Tibia': 'tibia', 'Shank': 'tibia', 'foot': 'tibia'}

# Solid densities in g/cm^3
PA12_CF = 1.06
PETG = 1.27
TPU = 1.21
BRASS = 8.5

# Printed parts weigh less than solid ones. A guess for walls plus infill, until the parts are weighed.
PRINT_FILL = 0.6

# Bought parts, grams per part (first match on the part's name). Estimates are marked; weigh and replace.
BOUGHT = [
    ('ST3215', 55.0),                    # Feetech ST3215 datasheet
    ('RASPBERRY_PI_5', 46.0),            # Raspberry Pi 5 datasheet
    ('RPLIDAR_A2M12', 190.0),            # Slamtec datasheet
    ('3S Battery Holder', 175.0),        # estimate: three 18650 cells and the holder with its BMS
    ('hexapod_breakout', 15.0),          # estimate: assembled board
    ('Bus Servo Adapter', 12.0),         # estimate
    ('Arducam_ToF_Camera', 10.0),        # estimate
    ('RPi_Camera_Module_3', 4.0),        # estimate
    ('Adafruit_ISM330DHCX', 2.0),        # estimate
]

# Printed parts, (name pattern, density). Leg parts are PA12-CF, the body and the rest PETG.
PRINTED = [
    (r'^foot', TPU),
    (r'HHTPS', BRASS),  # brass standoffs, solid metal so PRINT_FILL does not apply
    (r'^(Leg_mount|Coxa|Femur|Tibia|Shank|joint_cap)', PA12_CF),
    (r'.', PETG),
]

# Weighed printed parts, grams per part. Each part's density is scaled to match its weight.
MEASURED = {
    # 'Femur': 0.0,
}


@dataclass
class Solid:
    component: str  # top-level component, e.g. Femur
    part: str       # the part inside it, e.g. ST3215:2, or the component itself
    volume: float   # mm^3
    com: np.ndarray       # mm, CAD frame
    inertia: np.ndarray   # mm^5 (per unit density) about the centre of mass, CAD frame
    points: np.ndarray    # mesh vertices, mm, CAD frame


def label_name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ''


def strip_instance(name):
    return name.split(':')[0]


def read_step(path):
    """Return every solid in the STEP with its volume properties, placed in the CAD frame."""
    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != 1:
        raise SystemExit(f'Could not read {path}')
    reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    solids = []

    def walk(label, location, names):
        names = names + [label_name(label)]
        if shapes.IsReference_s(label):
            location = location.Multiplied(shapes.GetLocation_s(label))
            referred = TDF_Label()
            shapes.GetReferredShape_s(label, referred)
            label = referred
        if shapes.IsAssembly_s(label):
            children = TDF_ChildIterator(label, False)
            while children.More():
                if shapes.IsComponent_s(children.Value()):
                    walk(children.Value(), location, names)
                children.Next()
            return
        explorer = TopExp_Explorer(shapes.GetShape_s(label).Moved(location), TopAbs_SOLID)
        while explorer.More():
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(explorer.Current(), props)
            centre = props.CentreOfMass()
            matrix = props.MatrixOfInertia()
            solids.append(Solid(
                component=strip_instance(names[1]),
                part=names[2] if len(names) > 2 and not names[2].startswith('=>') else names[1],
                volume=props.Mass(),
                com=np.array([centre.X(), centre.Y(), centre.Z()]),
                inertia=np.array([[matrix.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]),
                points=mesh_points(explorer.Current())))
            explorer.Next()

    roots = TDF_ChildIterator(shapes.Label(), False)
    while roots.More():
        if shapes.IsFree_s(roots.Value()) and shapes.IsTopLevel(roots.Value()):
            walk(roots.Value(), TopLoc_Location(), [])
        roots.Next()
    return solids


def mesh_points(solid):
    BRepMesh_IncrementalMesh(solid, 0.5, False, 0.5, True)
    points = []
    faces = TopExp_Explorer(solid, TopAbs_FACE)
    while faces.More():
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(TopoDS.Face(faces.Current()), location)
        if triangulation is not None:
            transform = location.Transformation()
            for i in range(1, triangulation.NbNodes() + 1):
                p = triangulation.Node(i).Transformed(transform)
                points.append((p.X(), p.Y(), p.Z()))
        faces.Next()
    return np.array(points).reshape(-1, 3)


def assign_masses(solids):
    """Return each solid's density in g/mm^3, and a list of (part, grams, how) for the report."""
    groups = {}
    for solid in solids:
        groups.setdefault((solid.component, solid.part), []).append(solid)
    densities = {}
    report = []
    for (component, part), members in groups.items():
        volume = sum(s.volume for s in members)
        name = strip_instance(part)
        bought = next((grams for pattern, grams in BOUGHT if pattern in name), None)
        measured = MEASURED.get(name)
        if bought is not None:
            density, how = bought / volume, 'bought'
        elif measured is not None:
            density, how = measured / volume, 'weighed'
        else:
            solid_density = next(d for pattern, d in PRINTED if re.search(pattern, name))
            if solid_density == BRASS:
                density, how = solid_density / 1000.0, 'metal'
            else:
                density, how = solid_density * PRINT_FILL / 1000.0, 'printed'
        for solid in members:
            densities[id(solid)] = density
        report.append((LINKS[component], f'{component} / {part}', density * volume, how))
    return densities, report


def frame(origin, x, y):
    """Rotation (columns are the link's axes) and origin of a link frame given in base_link (mm)."""
    x = x / np.linalg.norm(x)
    y = y - x * np.dot(x, y)
    y = y / np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)]), origin


def fit_sphere(points):
    a = np.column_stack([2 * points, np.ones(len(points))])
    b = (points ** 2).sum(axis=1)
    solution = np.linalg.lstsq(a, b, rcond=None)[0]
    centre = solution[:3]
    return centre, math.sqrt(solution[3] + centre @ centre)


def to_base(points):
    return points @ CAD_TO_BASE.T - np.array([0.0, 0.0, HIP_HEIGHT])


def leg_frames(solids):
    """The STEP leg's coxa, femur and tibia frames in base_link."""
    def caps(component):
        return [to_base(s.com[None])[0] for s in solids
                if s.component == component and s.part.startswith('joint_cap')]

    coxa_axis = caps('Leg_mount')[0]
    femur_axis = np.mean(caps('Coxa'), axis=0)
    knee_axis = np.mean(caps('Tibia'), axis=0)
    foot = np.concatenate([to_base(s.points) for s in solids if s.component == 'foot'])
    # Only the lower half of the foot is ball-shaped
    foot_centre, foot_radius = fit_sphere(foot[foot[:, 2] < np.median(foot[:, 2])])

    outward = femur_axis - np.array([coxa_axis[0], coxa_axis[1], femur_axis[2]])
    up = np.array([0.0, 0.0, 1.0])
    side = np.cross(up, outward)  # the femur and knee axes, +y of every leg link
    coxa = frame(np.array([coxa_axis[0], coxa_axis[1], femur_axis[2]]), outward, side)
    femur = frame(femur_axis, knee_axis - femur_axis, side)
    tibia = frame(knee_axis, foot_centre - knee_axis, side)

    print('Leg measured from the joint caps (mm):')
    print(f'  coxa axis {np.hypot(*coxa_axis[:2]):.2f} from the centre, cap {coxa_axis[2]:.2f} above the hips')
    print(f'  coxa length {np.linalg.norm(femur_axis - coxa[1]):.2f}, femur length '
          f'{np.linalg.norm(knee_axis - femur_axis):.2f}, femur pitch '
          f'{math.degrees(math.atan2(knee_axis[2] - femur_axis[2], np.linalg.norm((knee_axis - femur_axis)[:2]))):.1f} deg')
    print(f'  foot ball radius {foot_radius:.2f}, knee to ball centre {np.linalg.norm(foot_centre - knee_axis):.2f}')
    if abs(np.hypot(*coxa_axis[:2]) - MIDDLE_Y) > 0.5:
        print(f'  NOTE: the URDF puts the middle coxa axes {MIDDLE_Y} mm from the centre. Update LEGS and the URDF.')
    return {'coxa': coxa, 'femur': femur, 'tibia': tibia}, coxa_axis


def yaw_transform(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]), np.array([x, y, 0.0])


def accumulate(items):
    """Combine (mass g, com mm, inertia g*mm^2 about its com) in one frame."""
    mass = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / mass
    inertia = np.zeros((3, 3))
    for m, c, i in items:
        d = c - com
        inertia += i + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, com, inertia


def link_properties(solids, densities, frames, cad_leg_axis):
    """Mass (kg), com (m) and inertia (kg*m^2) of each link, in its own frame."""
    items = {'base': [], 'coxa': [], 'femur': [], 'tibia': []}
    cad_leg = yaw_transform(cad_leg_axis[0], cad_leg_axis[1], CAD_LEG_YAW)
    for solid in solids:
        density = densities[id(solid)]  # g/mm^3
        mass = density * solid.volume
        com = to_base(solid.com[None])[0]
        inertia = CAD_TO_BASE @ (density * solid.inertia) @ CAD_TO_BASE.T
        link = LINKS[solid.component]
        if link == 'base' and solid.component == 'Leg_mount':
            # Move the STEP's one leg mount to each of the six coxa joints
            for leg in LEGS:
                rotation, origin = yaw_transform(*leg)
                r = rotation @ cad_leg[0].T
                items['base'].append((mass, r @ (com - cad_leg[1]) + origin, r @ inertia @ r.T))
        elif link == 'base':
            items['base'].append((mass, com, inertia))
        else:
            rotation, origin = frames[link]
            items[link].append((mass, rotation.T @ (com - origin), rotation.T @ inertia @ rotation))
    links = {}
    for link, parts in items.items():
        mass, com, inertia = accumulate(parts)
        links[link] = (mass / 1e3, com / 1e3, inertia / 1e9)
    return links


def mirrored(properties):
    """The same link on the other side of the robot (y -> -y)."""
    mass, com, inertia = properties
    flip = np.diag([1.0, -1.0, 1.0])
    return mass, flip @ com, flip @ inertia @ flip


def inertial_xml(properties, indent):
    mass, com, i = properties
    pad = ' ' * indent
    return (f'{pad}<inertial>\n'
            f'{pad}  <mass value="{mass:.4f}"/>\n'
            f'{pad}  <origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}"/>\n'
            f'{pad}  <inertia ixx="{i[0, 0]:.4e}" ixy="{i[0, 1]:.4e}" ixz="{i[0, 2]:.4e}" '
            f'iyy="{i[1, 1]:.4e}" iyz="{i[1, 2]:.4e}" izz="{i[2, 2]:.4e}"/>\n'
            f'{pad}</inertial>\n')


def write_xacro(path, step_name, links, report):
    total = links['base'][0] + 6 * sum(links[k][0] for k in ('coxa', 'femur', 'tibia'))
    lines = [
        '<?xml version="1.0"?>',
        f'<!-- Generated by scripts/mass_properties.py from {step_name}, edit that instead.',
        f'     Whole robot {total:.3f} kg. Printed parts are at {PRINT_FILL:.0%} of solid until weighed. -->',
        '<robot xmlns:xacro="http://ros.org/wiki/xacro">',
        '',
        '  <xacro:macro name="hexapod_base_inertial">',
        inertial_xml(links['base'], 4).rstrip('\n'),
        '  </xacro:macro>',
        '',
        '  <!-- link: coxa, femur or tibia; side: left or right -->',
        '  <xacro:macro name="hexapod_leg_inertial" params="link side">',
    ]
    for link in ('coxa', 'femur', 'tibia'):
        for side, properties in (('left', links[link]), ('right', mirrored(links[link]))):
            lines.append(f"    <xacro:if value=\"${{link == '{link}' and side == '{side}'}}\">")
            lines.append(inertial_xml(properties, 6).rstrip('\n'))
            lines.append('    </xacro:if>')
    lines += ['  </xacro:macro>', '', '</robot>', '']
    path.write_text('\n'.join(lines))

    print(f'\nWrote {path}, whole robot {total * 1000:.0f} g')
    for link in ('base', 'coxa', 'femur', 'tibia'):
        mass, com, inertia = links[link]
        print(f'  {link:6s} {mass * 1000:7.1f} g  com {np.round(com * 1000, 1)} mm  '
              f'principal inertia {np.round(np.linalg.eigvalsh(inertia) * 1e6, 1)} kg*mm^2')
    print('\nParts (grams):')
    for link, part, grams, how in sorted(report, key=lambda r: (r[0], -r[2])):
        if grams >= 1.0:
            print(f'  {link:6s} {grams:7.1f}  {how:8s} {part}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('step', type=Path)
    parser.add_argument('--output', type=Path, default=PACKAGE_DIR / 'urdf' / 'hexapod.inertials.xacro')
    args = parser.parse_args()

    solids = [s for s in read_step(args.step) if s.component in LINKS]
    densities, report = assign_masses(solids)
    frames, cad_leg_axis = leg_frames(solids)
    links = link_properties(solids, densities, frames, cad_leg_axis)
    write_xacro(args.output, args.step.name, links, report)


if __name__ == '__main__':
    main()
