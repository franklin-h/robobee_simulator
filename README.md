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