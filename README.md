# Deformable Object Manipulation Sandbox

Our goal is to study and compare how deformable object manipulation is simulated in 2 popular physics engines, Bullet and Mujoco. Mainly, we would like to understand the contact physics and evaluate any differences, which would be crucial for controlling manipulators, espeically in real-world settings, where there might be a lot of unknowns regarding the contact forces. 

We hope that this sandbox can help users:
* Have a baseline understanding on how the coding (i.e. PyBullet vs native Mujoco python bindings) and model file definitions (i.e. MJCF vs URDF) would differ between these frameworks. 
* Understand about how physics and control differ between these engines in the context of deformable object manipulation.
* Pick a more suitable simulator for their use case, based on the results presented.
* Run experiments comparing the 2 simulators

## Installation

Note: This sandbox is tested on a machine with Ubuntu 18.04 installed.

To install the package with its dependencies, run
```
pip install .
```

Alternatively, you can use `poetry`
```
pip install --upgrade pip
pip install poetry
poetry install
```

You might also need to install glfw for OpenGL-based rendering (particularly for mujoco)

```
sudo apt-get install libglfw3
```

## Running the code

### Mujoco

Run the following commands (args are optional with default values set):
```
cd mujoco
python3 mujoco_main.py --control_mode <control_mode> --xml_path <xml_path>
```

By default, this loads the `cloth_panda.xml` model file, which contains a Panda manipulator and a cloth, into a scene rendered with OpenGL.

Expected output (TODO use gif/video):
<img src="./doc_assets/mj_cloth_panda.png " alt="drawing" width="800"/>

<!-- For using Panda
```
python3 native_mujoco.py --control_mode <control_mode> --xml_path <xml_path>
``` -->

<!-- Using mujoco-viewer
```
python3 mujoco_with_viewer.py --xml_path data/franka_emika_panda/scene.xml
``` -->

### PyBullet

Run the following commands.

```
cd pybullet
python3 bullet_main.py --manipulator_xml_path <manipulator_xml_path> --cloth_xml_path <cloth_xml_path>
```

This loads the `panda.urdf` via `loadURDF` and `cloth_z_up.obj` with `loadSoftBody`. 

The code is mostly derived from the [deformable anchor](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py) pybullet example. 

Expected output (TODO use gif/video):
<img src="./doc_assets/bullet_start.png " alt="drawing" width="800"/>

An issue: At render, the cloth is only visible from below (thus placed above the manipulator for now).  

<!-- <img src="./doc_assets/bullet_end.png " alt="drawing" width="800"/> -->
## Notes

### General 
* Mujoco bodies need to all be compiled within XML, can't be dynamically loaded, unlike in PyBullet. 
* Refer to Notion page for further notes

### Mujoco 
* The Python bindings are almost the same as the C bindings in which the documentation can be found [here](https://mujoco.readthedocs.io/en/latest/APIreference.html).

* [Soft contacts and physical realism](https://mujoco.readthedocs.io/en/latest/computation.html#physical-realism-and-soft-contacts)

* Working manipulators (relative to `mujoco` folder)
    * `data/franka_emika_panda/panda.xml`from [mujoco_menagerie](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
    * `data/mjcf_ur5e/ur5e.xml`from [mujoco_menagerie](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda)

* Soft bodies + Manipulator
    * `data/cloth_gripper.xml`
    * `data/cloth.xml`
    * `data/cloth_panda/cloth_panda.xml`

* Using `mujoco-python-viewer` for rendering for now since it wraps `glfw` functions well and provides an interactive render. Will study the base code further to see if we can remove this dependency. 

### PyBullet

A soft/deformable body can be loaded from either an `.obj` mesh file, using loadSoftBody (with the physics defined on the function call itself) or a `urdf` file (physics defined in the urdf file itself).

Working manipulators (relative to `pybullet` folder)
* `data/ur_description/urdf/ur10_robot.urdf`from [example-robot-data](https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description)
* `data/franka_panda/panda.urdf` from [bullet repo](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data/franka_panda)

Soft bodies
* `data/bullet_official/cloth_z_up.obj`

