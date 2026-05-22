# CAD Symmetry Fix

The original OnShape CAD for Open Duck Mini V2 had a significant bilateral asymmetry in the right leg. This was discovered during training analysis, traced to specific mate connector placements in the CAD, and fixed at the source.

---

## 1. Discovery

During policy evaluation, trained policies consistently showed asymmetric behaviour - better left turns than right turns, asymmetric gait patterns, and a gait symmetry score of 0.25 (where 1.0 is perfect). Initial investigation assumed this was a training artefact.

Analysis of the exported MJCF model revealed the asymmetry was physical: the right lower leg bodies were offset by approximately 37mm in the z-axis compared to where they should be if the right leg mirrored the left.

The offset propagated through the entire downstream pipeline:
- The MJCF model had asymmetric body positions
- The URDF used by Placo had the same asymmetry
- Reference motions generated from the URDF were asymmetric
- Policies trained with asymmetric reference motions on an asymmetric model learned asymmetric behaviour

## 2. Root Cause

The issue was in the OnShape mate connector placements for the right leg. When the right leg subassemblies were originally added to the main assembly, the mate connectors were not placed to produce a true mirror of the left leg.

Specific problems found:
- **Mate entity order**: OnShape mates define parent-child relationships. The first entity in a mate becomes the parent in the kinematic chain. Several right-leg mates had the entity order swapped compared to their left-leg equivalents
- **Mate naming**: The `onshape-to-robot` exporter uses NAME PREFIXES to determine export behaviour. `dof_` prefix creates a revolute joint, `fix_` prefix creates a fixed (welded) connection. Some right-leg mates were missing these prefixes. The actual OnShape mate type (Fastened/Revolute) does not affect the export - only the name prefix matters.
- **Missing frame**: The `frame_right_foot` site (used for foot contact sensing) was not present in the assembly

## 3. Fix Applied

All right-leg mates were recreated from scratch in OnShape:

1. Deleted all existing right-leg mates
2. Created new mates with correct entity order (parent body first, child body second, matching the left leg convention)
3. Named all mates with correct prefixes (`dof_` for revolute joints, `fix_` for fixed connections)
4. Added the missing `frame_right_foot` mate
5. Antenna mates kept as Revolute in OnShape but named with `fix_` prefix - this allows the antennas to flex physically while being welded to the head in the MJCF export

## 4. Verification

After re-exporting via `onshape-to-robot`:

**Body positions**: All bilateral body pairs now have 0.000mm positional difference (previously up to 37mm).

**Joint axes**: True mirror convention confirmed. Examples:
- `left_knee` axis: [0, -1, 0]
- `right_knee` axis: [0, +1, 0]

**Home keyframe**: Recalculated so right leg values are exact mirrors of left. Both feet sit at identical height (0.000mm z difference).

**Reference motions**: All 210 motions regenerated with the symmetric URDF. Visual playback of forward, backward, strafe, and turning motions confirmed correct behaviour.

## 5. OnShape-to-MJCF Export Conventions

For future reference, the `onshape-to-robot` export pipeline requires:

| Convention | Meaning |
|-----------|---------|
| `dof_` prefix on mate name | Exported as a revolute joint |
| `fix_` prefix on mate name | Exported as a fixed (welded) connection |
| `frame_` prefix on mate name | Exported as a site (sensor attachment point) |
| Mate entity order | First entity = parent body in kinematic chain |
| Mate type vs name | The NAME prefix controls export behaviour, NOT the OnShape mate type. A mate can be Revolute in OnShape but exported as fixed if named `fix_*`. This is how antennas work (Revolute in OnShape so they flex, but `fix_*` named so they're welded in MJCF) |
| Fixed-joint body merging | Bodies connected by `fix_` mates are merged into one body in MJCF. This is why `right_cache` appears as a body name - it is the merged upper_leg + cache assembly, which is functionally correct |

## 6. Files Changed

| File | Change |
|------|--------|
| `Open_Duck_Playground/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml` | New symmetric MJCF export |
| `Open_Duck_Playground/playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml` | Updated home keyframe with mirrored right leg values |
| `Open_Duck_reference_motion_generator/robots/open_duck_mini_v2/open_duck_mini_v2.urdf` | New symmetric URDF export |
| `Open_Duck_Playground/playground/open_duck_mini_v2/data/polynomial_coefficients.pkl` | Regenerated from symmetric reference motions |
| `Open_Duck_Playground/playground/open_duck_mini_v2/xmls.backup/` | Backup of old asymmetric files |
