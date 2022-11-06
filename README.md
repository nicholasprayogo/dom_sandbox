# Deformable Object Manipulation Sandbox

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
python3 native_mujoco.py --control_mode <control_mode> --xml_path <xml_path>
```

This loads the `cloth_gripper.xml` file, which contains a manipulator and a cloth, into a scene rendered with OpenGL, which is the primary renderer supported by Mujcoo.

Expected output (TODO use gif/video):
<img src="./doc_assets/mjc_freefall_start.png " alt="drawing" width="800"/>
<img src="./doc_assets/mjc_freefall_end.png " alt="drawing" width="800"/>

### PyBullet

Run the following commands.

```
cd pybullet
python3 dom_bullet.py
```

This loads the `panda.urdf` via `loadURDF` and `cloth_z_up.obj` with `loadSoftBody`. 

The code is mostly from the [deformable anchor](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py) pybullet example

Expected output (TODO use gif/video):
<img src="./doc_assets/bullet_start.png " alt="drawing" width="800"/>
<img src="./doc_assets/bullet_end.png " alt="drawing" width="800"/>
## Notes

### Mujoco 

### PyBullet

A soft/deformable body can be loaded from either an `.obj` mesh file, using loadSoftBody (with the physics defined on the function call itself) or a `urdf` file (physics defined in the urdf file itself).