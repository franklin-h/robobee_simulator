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