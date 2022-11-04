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

1. Mujoco

```
cd mujoco
python3 native_mujoco.py
```

2. PyBullet

```
cd pybullet
python3 dom_bullet.py
```

## Notes