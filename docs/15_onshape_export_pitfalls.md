# OnShape Export Pitfalls and How to Avoid Them

When re-exporting the robot from OnShape via `onshape-to-robot`, the resulting MJCF needs several modifications before it can be used for MJX training. This document records the exact issues encountered and how to handle them in future.

---

## 1. The Six Issues

### Issue 1: Missing geom names for foot contact

**Symptom**: Training crashes immediately with `KeyError: "Invalid name 'left_foot_bottom_tpu'. Valid names: ['floor']"`

**Cause**: The `onshape-to-robot` exporter doesn't add `name=` attributes to geoms. Only bodies, joints, and sites get named. The training code (`constants.py`) expects `left_foot_bottom_tpu` and `right_foot_bottom_tpu` for floor contact detection in the reward function.

**Fix**: Add `name="left_foot_bottom_tpu"` and `name="right_foot_bottom_tpu"` attributes to the foot TPU collision geoms (the `<geom class="collision">` lines).

**Why these specific names matter**: The reward function does `geom_id = model.geom("left_foot_bottom_tpu").id` to determine when each foot is touching the floor. Without names, the lookup fails. The names themselves are arbitrary but must match `constants.FEET_GEOMS`.

### Issue 2: Missing `base` wrapper body

**Symptom**: Training compiles but generates a different (larger) XLA computation graph that triggers a `ptxas` segfault on RTX 5080 (CUDA 13.0).

**Cause**: The exporter places the freejoint directly on `trunk_assembly`. The training pipeline expects a `base` body that wraps `trunk_assembly`, with the freejoint and IMU site on `base`, not on `trunk_assembly`.

**Fix**: Wrap the `trunk_assembly` body inside a `base` body:
```xml
<worldbody>
    <body name="base" pos="0 0 0.22">
        <freejoint name="floating_base"/>
        <site name="imu" pos="-0.08 -0.0 0.05"/>
        <body name="trunk_assembly" ... >
            <!-- ... -->
        </body>
    </body>
</worldbody>
```

**Why this matters**: The kinematic chain depth and body count affect how MJX vectorises the simulation. The wrapper structure produces a graph that `ptxas` can compile; the flat structure produces a graph that crashes the compiler.

### Issue 3: Antenna joints exported as revolute

**Symptom**: 16 actuators instead of 14, observation size becomes 113 instead of 101.

**Cause**: The exporter creates `<joint type="hinge">` for antenna mates that are defined as Revolute in OnShape.

**Fix at the source (recommended)**: Name the antenna mates with the `fix_` prefix (e.g., `fix_left_antenna`, `fix_right_antenna`). The `onshape-to-robot` exporter uses the mate NAME PREFIX to decide joint type, regardless of the actual mate type in OnShape:
- `dof_<name>` → exports as a revolute joint
- `fix_<name>` → exports as a fixed connection (parts get merged into the parent body)

So the antenna mates can stay as Revolute in OnShape (which makes sense for a physical antenna that flexes) - what matters is the NAME prefix. With `fix_` prefix, the exporter merges the antenna parts into `head_assembly` automatically - no joints, no separate bodies, but visual meshes are preserved. This matches the original Open Duck Mini design and the working old MJCF.

**Post-export fix (fallback)**: If for some reason antennas must remain Revolute in OnShape (e.g., for a future antenna-control policy), remove the antenna bodies, joints, and actuators manually. The patch script can do this automatically but it loses the visual antennas.

**Why this matters**: Walking policies don't control antennas. Adding antenna joints increases the action space (more DOFs to control), the observation space (more joint positions/velocities), and the XLA graph size - all without any benefit. If/when antenna control becomes a goal, antennas should be controlled by a separate policy or scripted, not by the walking policy.

### Issue 4: Missing solver options

**Symptom**: `ptxas` segfaults during JIT compilation with "Registers are spilled to local memory" warnings.

**Cause**: MJX uses MuJoCo's default solver settings (~20+ iterations) which produces a huge unrolled computation graph. The original working model used aggressive solver simplification.

**Fix**: Add to the top of the MJCF, after `<mujoco>`:
```xml
<option iterations="1" ls_iterations="5">
    <flag eulerdamp="disable"/>
</option>
```

**Why this matters**: 
- `iterations="1"`: Limits constraint solver to 1 iteration (default is higher)
- `ls_iterations="5"`: Limits line search to 5 iterations
- `eulerdamp="disable"`: Skips Euler damping in the integrator

These reduce the per-step computation graph by ~10x, allowing `ptxas` to compile it for RTX 5080. The simulation is slightly less physically accurate but adequate for RL training.

### Issue 5: Asymmetric body naming

**Symptom**: Cosmetic - the right leg has a body called `right_cache` while the left leg has equivalents named `knee_and_ankle_assembly` and `_2`.

**Cause**: The exporter names merged bodies after the OnShape part that contains the fixed mate. Different mate naming on left vs right produces different body names even when the structure is functionally symmetric.

**Fix**: Rename for visual consistency:
- `right_cache` -> `knee_and_ankle_assembly_3`
- Existing `knee_and_ankle_assembly_3` -> `knee_and_ankle_assembly_4`

Do NOT rename the mesh (`right_cache.stl`) or material (`right_cache_material`) - only the body name.

**Why this matters**: Purely cosmetic. The training code never references these names. But matching the left side naming makes the model easier to read and matches the old working model exactly.

### Issue 6: Home keyframe must match joint count

**Symptom**: After making changes to joint count, training crashes with cryptic position errors, or the robot's feet sit at wildly different heights (e.g., 77mm difference).

**Cause**: The home keyframe in `scene_flat_terrain.xml` has a fixed-length `qpos` and `ctrl` string. If you add or remove joints, the keyframe values shift by position, breaking the joint-to-value mapping.

**Fix**: After any joint count change, recalculate the keyframe. The qpos format is:
- `qpos[0:3]`: base xyz (e.g., `0 0 0.15`)
- `qpos[3:7]`: base quaternion wxyz (e.g., `1 0 0 0`)
- `qpos[7:]`: joint positions in joint definition order

Right-leg values should be exact negations of left-leg values for the joints that are mirror-symmetric (e.g., right_knee = -left_knee for the duck's mirror convention).

**Verify**: After updating, foot heights should be within 0.5mm of each other:
```python
data.qpos[:] = model.keyframe('home').qpos
mujoco.mj_forward(model, data)
lz = data.site_xpos[model.site('left_foot').id][2]
rz = data.site_xpos[model.site('right_foot').id][2]
assert abs(lz - rz) < 0.001
```

---

## 2. The Full Fix Checklist (after any re-export)

When you re-export the robot from OnShape, apply these changes to `open_duck_mini_v2.xml` in order:

1. **Add solver options** after `<mujoco ...>` opening tag
2. **Wrap trunk in base body**: add `<body name="base" pos="0 0 0.22">` with freejoint and imu site
3. **Remove imu site** from `trunk_assembly` (it's now on `base`)
4. **Close base body**: add extra `</body>` before `</worldbody>`
5. **Delete antenna bodies** entirely (or move their visual geoms to head_assembly)
6. **Remove antenna actuators** from `<actuator>` block
7. **Add name attribute** to foot TPU collision geoms: `name="left_foot_bottom_tpu"` and `name="right_foot_bottom_tpu"`
8. **Rename right_cache** body to `knee_and_ankle_assembly_3`, rename existing `knee_and_ankle_assembly_3` to `knee_and_ankle_assembly_4`

Then in `scene_flat_terrain.xml`:

9. **Update home keyframe** if joint count changed: remove antenna values, ensure right leg values are exact negations of left
10. **Update ctrl values** to match new actuator count

---

## 3. How to Avoid This in Future

### Option A: Post-export patching script (recommended)

Write a Python script that takes the raw `onshape-to-robot` output and applies all the fixes deterministically:

```python
#!/usr/bin/env python3
"""patch_mjcf.py - Apply training-compatible fixes to onshape-to-robot output."""
import xml.etree.ElementTree as ET
import sys

def patch_mjcf(input_path, output_path):
    tree = ET.parse(input_path)
    root = tree.getroot()
    
    # 1. Add solver options
    option = ET.SubElement(root, 'option', iterations='1', ls_iterations='5')
    ET.SubElement(option, 'flag', eulerdamp='disable')
    root.insert(1, option)  # near the top
    
    # 2-4. Wrap trunk in base body
    worldbody = root.find('worldbody')
    trunk = worldbody.find("body[@name='trunk_assembly']")
    base = ET.Element('body', name='base', pos='0 0 0.22')
    ET.SubElement(base, 'freejoint', name='floating_base')
    ET.SubElement(base, 'site', name='imu', pos='-0.08 0 0.05')
    worldbody.remove(trunk)
    base.append(trunk)
    worldbody.append(base)
    
    # Remove imu site from trunk
    for site in trunk.findall("site[@name='imu']"):
        trunk.remove(site)
    
    # 5. Delete antenna bodies
    for head in root.iter('body'):
        if head.get('name') == 'head_assembly':
            for child in list(head):
                if child.tag == 'body' and 'antenna' in child.get('name', ''):
                    head.remove(child)
    
    # 6. Remove antenna actuators
    actuator = root.find('actuator')
    for pos in actuator.findall('position'):
        if 'antenna' in pos.get('name', ''):
            actuator.remove(pos)
    
    # 7. Name foot collision geoms
    for body in root.iter('body'):
        name = body.get('name', '')
        if name == 'foot_assembly':
            for geom in body.findall('geom'):
                if geom.get('class') == 'collision':
                    geom.set('name', 'left_foot_bottom_tpu')
        elif name == 'foot_assembly_2':
            for geom in body.findall('geom'):
                if geom.get('class') == 'collision':
                    geom.set('name', 'right_foot_bottom_tpu')
    
    # 8. Rename right_cache and knee_and_ankle_assembly_3
    for body in root.iter('body'):
        if body.get('name') == 'knee_and_ankle_assembly_3':
            body.set('name', 'knee_and_ankle_assembly_4')
    for body in root.iter('body'):
        if body.get('name') == 'right_cache':
            body.set('name', 'knee_and_ankle_assembly_3')
    
    tree.write(output_path)

if __name__ == '__main__':
    patch_mjcf(sys.argv[1], sys.argv[2])
```

Run it after every export:
```bash
onshape-to-robot config.json
python patch_mjcf.py raw_export.xml open_duck_mini_v2.xml
```

This is reproducible, version-controllable, and doesn't require maintaining a fork of `onshape-to-robot`.

### Option B: Fork onshape-to-robot

The exporter is at https://github.com/Rhoban/onshape-to-robot. You could fork it and:
- Add config options for geom naming patterns
- Add hooks to inject custom XML elements
- Add support for "merge child body into parent" mate annotations

**Pros**: Cleaner pipeline, fixes flow back to upstream if accepted.
**Cons**: Maintaining a fork requires periodic rebasing. Most of these issues are project-specific (the `base` wrapper, the solver options, the specific naming conventions) and unlikely to be accepted upstream.

**Verdict**: Not worth it for this project. The post-export patching script gives the same result with much less maintenance burden.

### Option C: Modify OnShape mate setup

For some issues, you can avoid the problem at the source:
- **Antenna mates**: Use Fastened (not Revolute) for antennas in OnShape so they don't generate joints
- **Geom names**: Cannot be controlled from OnShape - the exporter doesn't add them regardless

This only helps with antennas. The other issues are exporter-level decisions that can't be configured from OnShape.

---

## 4. Recommended Workflow

```
1. Edit CAD in OnShape:
   - Geometry, masses, joint mates as needed
   - Walking joint mates: Revolute type, named with `dof_` prefix (e.g., `dof_left_knee`)
   - Antenna mates: named with `fix_` prefix (e.g., `fix_left_antenna`) - the mate
     itself can stay Revolute in OnShape, the `fix_` prefix tells the exporter to
     merge the part into the parent body
   - Foot frame mates: named with `frame_` prefix (e.g., `frame_left_foot`)
   - Mate entity order: parent body first, child body second (matters for kinematic chain)
2. Run onshape-to-robot to export raw MJCF + STL files
3. Run patch_mjcf.py to apply training-compatibility fixes (issues 1, 2, 4, 5)
4. Manually verify with: 
   - Load model in MuJoCo viewer (check visual appearance)
   - Check foot heights symmetric in home keyframe
   - Check constants.py references all resolve
5. Run a 1M step training smoke test to verify ptxas compiles
6. If smoke test passes, run full 150M training
```

**Note on antenna handling**: If antennas are Fastened in OnShape, the patch script does NOT need to remove antenna bodies (they don't get created). This is the preferred approach. The script's antenna-removal logic only runs if antenna bodies are detected, so it's safe to leave it enabled.

For the home keyframe specifically, automate the symmetry check:

```python
# In scene_flat_terrain.xml keyframe maintenance
def verify_keyframe_symmetry(model_path):
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    data.qpos[:] = model.keyframe('home').qpos
    mujoco.mj_forward(model, data)
    lz = data.site_xpos[model.site('left_foot').id][2]
    rz = data.site_xpos[model.site('right_foot').id][2]
    assert abs(lz - rz) < 0.001, f'Asymmetric feet: L={lz}, R={rz}'
```

---

## 5. Build the Patch Script Now?

Yes - we should build `patch_mjcf.py` properly as a Python script in `Open_Duck_Playground/scripts/` so any future re-export uses it automatically. This formalises the fixes and prevents drift between re-exports.

The script should be deterministic (same input always produces same output) and validatable (run model checks at the end).

---

## 6. Sources

- `onshape-to-robot`: https://github.com/Rhoban/onshape-to-robot
- MuJoCo options reference: https://mujoco.readthedocs.io/en/stable/XMLreference.html#option
- MJX solver iteration trade-offs: https://mujoco.readthedocs.io/en/stable/mjx.html
- ptxas register spilling: NVIDIA CUDA documentation, error code 139 = SIGSEGV
