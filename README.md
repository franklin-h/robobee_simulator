# RoboBee Model 
I incorporated the useful parts into the Bazel project as model data:

```text
models/
  BUILD.bazel
  wing_asy/
    BUILD.bazel
    package.xml
    urdf/wing_asy.urdf
    meshes/Part_1.stl
    meshes/Part_1_1.stl
```

The important Bazel target is:

```python
//models/wing_asy:files
```

defined in [models/wing_asy/BUILD.bazel](/Users/franklinho/robobee_simulator/models/wing_asy/BUILD.bazel). I verified Bazel sees it with:

```bash
bazel query //models/wing_asy:*
```

To use it from an app, add it as runtime data:

```python
cc_binary(
    name = "hello_world",
    srcs = ["hello_world.cc"],
    data = [
        "//models/wing_asy:files",
    ],
)
```

Then Drake code can load the URDF, but there is one key detail: your URDF references meshes like this:

```xml
package://wing_asy/meshes/Part_1.stl
```

So when parsing it in Drake, you must register a package map entry named `wing_asy` pointing at `models/wing_asy`.

Conceptually:

```cpp
drake::multibody::Parser parser(&plant);
parser.package_map().Add("wing_asy", path_to_models_wing_asy);
parser.AddModelsFromUrl("package://wing_asy/urdf/wing_asy.urdf");
```

The `launch/wing_asy.launch` file is ROS-specific. For a Drake/Bazel project, you usually do not use it unless you are also running ROS/RViz. The URDF and meshes are the parts Drake needs.

## Creating the URDF Assembly

Created the FWMAV assembly package at [models/fwmav_asy](/Users/franklinho/robobee_simulator/models/fwmav_asy).

Main URDF: [fwmav_asy.urdf](/Users/franklinho/robobee_simulator/models/fwmav_asy/urdf/fwmav_asy.urdf)

What it includes:
- Fuselage as a `0.00228 x 0.00228 x 0.01335` m box, with `z` vertical.
- Left and right wing meshes reused from `wing_asy` and `wing_asy_right`.
- Each wing mounted through two serial revolute joints:
  - yaw about `z`
  - pitch about `y`
- Hinge origins placed at opposite top-face vertices:
  - left: `-0.00114 0.00114 0.006675`
  - right: `0.00114 -0.00114 0.006675`
- Added `package.xml` dependencies and Bazel visibility via [BUILD.bazel](/Users/franklinho/robobee_simulator/models/fwmav_asy/BUILD.bazel).

Verified with:
- `xmllint --noout ...`
- `bazel build //models/fwmav_asy:files`

I think OBJ works better than STL, otherwise you can't preview. 

Don't forget: when importing the URDF for the wings, make sure the COM is coincident with the origin of the CAD. (this is not relevant anymore) 


## Simplified CAD for URDF: 
- Take, say, the transmission. Export as a STEP file. 
- Then, we need to simplify. Split along the middle of the capton. Then use boolean to combine various layers so they are just simplified linkage blocks. 
- Then reimport into an assembly, and add revolute joints. 
- When you put it back in main CAD, be careful about selecting faces for mates (such as Planar mates with offset. Boolean operation can create fucked up faces )
- \In case mates stll fuck up: Might also try to do planar first for transmission base, move the slider closer to where it should be, THEN add slider.  
- you can also fix the entire airframe so it is mechanical ground. each part of airframe must be fixed! 
- start an restart meldis can help lmao 

## ```visualize_robobee_assembly.cc``` explained. 
Because “one Onshape subassembly” is not the same thing as “one serial kinematic chain.”

A subassembly is just a grouping of parts/mates in CAD. The robot model needs a **tree of links and joints**. If the CAD mates form a closed linkage, the exporter has to break that graph somewhere, because URDF/Drake cannot represent a parent-child cycle directly.

For your left transmission, the physical idea is more like a loop:

```text
fixed airframe/base
  -> slider input
  -> transmission_link_1
  -> transmission_link_2
  -> transmission_hinge
  -> transmission_base
  -> fixed airframe/base
```

But URDF must encode it as an open tree, so it gets split into branches:

```text
root/airframe branch
  -> part_1_3
  -> slider_2
  -> transmission_link_1
  -> transmission_link_2
```

and:

```text
root
  -> transmission_base
  -> transmission_hinge
  -> transmission_link_2__1__loop_closure
```

That fake `__loop_closure` link is the exporter saying: “this end should connect back to another part, but I cannot encode the cycle here.”

Then your C++ IK adds the missing physical constraint:

```cpp
transmission_link_2__1__loop_closure == transmission_link_2
```

So the split is not because Onshape thinks they are unrelated. It is because the transmission is a closed mechanism, and the exporter has to cut closed mechanisms into a URDF-compatible tree. The C++ code then closes the cut mathematically.

Yes, that’s the right summary.

More precisely:

```text
Onshape assembly: mechanically closed linkage
URDF export: open tree with one or more artificial loop_closure ends
Drake IK: adds constraints that force those artificial ends back onto the matching real links
```

And yes, “closed” here means the mechanism forms a loop back to the fixed structure/root, roughly:

```text
root / airframe
  -> slider
  -> transmission_link_1
  -> transmission_link_2
  -> transmission_hinge
  -> transmission_base
  -> root / airframe
```

The URDF cannot directly encode that cycle, so it cuts the loop and leaves a free-looking end like:

```text
transmission_link_2__1__loop_closure
```

Your C++ code closes it by constraining that frame to coincide with:

```text
transmission_link_2
```

So the solver is not inventing the closure; it is restoring the physical mate relationship that had to be cut for URDF compatibility.


## Proper Multibody Sim in ```visualize_robobee_linkage```
Drafted the new constrained linkage visualizer in [apps/visualize_robobee_linkage.cc](/Users/franklinho/robobee_simulator/apps/visualize_robobee_linkage.cc) and added the Bazel target in [apps/BUILD.bazel](/Users/franklinho/robobee_simulator/apps/BUILD.bazel:55).

What it does:
- Loads `robobee_assembly.urdf`.
- Uses a discrete `MultibodyPlant` with SAP-compatible constraints.
- Closes the two transmission loops using `MultibodyPlant::AddWeldConstraint`, not per-frame IK.
- Adds PD-controlled actuators on `slider_1` and `slider_2`.
- Drives both sliders with a sinusoidal desired stroke source.
- Publishes to Meldis like the existing visualizers.

Verified:
- `bazel build //apps:visualize_robobee_linkage`
- Short `bazel run //apps:visualize_robobee_linkage` smoke test reached simulation startup and publishing.

Run it with:

```bash
bazel run //apps:visualize_robobee_linkage
```

The PD gains and actuator force limits are starter visualization values, not calibrated RoboBee actuator parameters yet. Also, I left the existing modified `apps/visualize_robobee_assembly.cc` untouched.