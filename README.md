# Deformable Object Manipulation Sandbox

Our goal is to study and compare how deformable object manipulation is simulated in 2 popular physics engines, Bullet and Mujoco. Mainly, we would like to understand how the contact physics work and perhaps differ between the 2.

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
python3 mujoco_with_viewer.py --control_mode <control_mode> --xml_path <xml_path>
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
python3 bullet_sandbox.py
```

This loads the `panda.urdf` via `loadURDF` and `cloth_z_up.obj` with `loadSoftBody`. 

The code is mostly derived from the [deformable anchor](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py) pybullet example. 

Expected output (TODO use gif/video):
<img src="./doc_assets/bullet_start.png " alt="drawing" width="800"/>

<!-- <img src="./doc_assets/bullet_end.png " alt="drawing" width="800"/> -->
## Notes

### General 
* Mujoco bodies need to all be compiled within XML, can't be dynamically loaded, unlike in PyBullet. 

### Mujoco 
* The Python bindings are almost the same as the C bindings in which the documentation can be found [here](https://mujoco.readthedocs.io/en/latest/APIreference.html).

* [Soft contacts and physical realism](https://mujoco.readthedocs.io/en/latest/computation.html#physical-realism-and-soft-contacts)

Working manipulators (relative to `mujoco` folder)
* `data/franka_emika_panda/panda.xml`from [mujoco_menagerie](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
* `data/mjcf_ur5e/ur5e.xml`from [mujoco_menagerie](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda)

Soft bodies + Manipulator
* `data/cloth_gripper.xml`
* `data/cloth.xml`
* `data/cloth_panda/cloth_panda.xml`


### PyBullet

A soft/deformable body can be loaded from either an `.obj` mesh file, using loadSoftBody (with the physics defined on the function call itself) or a `urdf` file (physics defined in the urdf file itself).

Working manipulators (relative to `pybullet` folder)
* `data/ur_description/urdf/ur10_robot.urdf`from [example-robot-data](https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description)
* `data/franka_panda/panda.urdf` from [bullet repo](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data/franka_panda)

Soft bodies
* `data/bullet_official/cloth_z_up.obj`

