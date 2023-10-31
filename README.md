# Omniverse MJCF Importer

The MJCF Importer Extension is used to import MuJoCo representations of scenes.
MuJoCo Modeling XML File (MJCF), is an XML format for representing a scene in the MuJoCo simulator.

## Getting Started

1. Clone the GitHub repo to your local machine.
2. Open a command prompt and navigate to the root of your cloned repo.
3. Run `build.bat` to bootstrap your dev environment and build the example extensions.
4. Run `_build\{platform}\release\omni.importer.mjcf.app.bat` to start the Kit application.
5. From the menu, select `Isaac Utils->MJCF Importer` to launch the UI for the MJCF Importer extension.

This extension is enabled by default. If it is ever disabled, it can be re-enabled from the Extension Manager
by searching for `omni.importer.mjcf`.

**Note:** On Linux, replace `.bat` with `.sh` in the instructions above.


## Conventions

Special characters in link or joint names are not supported and will be replaced with an underscore. In the event that the name starts with an underscore due to the replacement, an a is pre-pended. It is recommended to make these name changes in the mjcf directly.

See the [Convention References](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_conventions.html#isaac-sim-conventions) documentation for a complete list of `Isaac Sim` conventions.


## User Interface

![MJCF Importer UI](/images/importer_mjcf_ui.png)

### Configuration Options

* **Fix Base Link**: When checked, every world body will have its base fixed where it is placed in world coordinates.
* **Import Inertia Tensor**: Check to load inertia from mjcf directly. If the mjcf does not specify an inertia tensor, identity will be used and scaled by the scaling factor. If unchecked, Physx will compute it automatically.
* **Stage Units Per Meter**: The default length unit is meters. Here you can set the scaling factor to match the unit used in your MJCF.
* **Link Density**: If a link does not have a given mass, it uses this density (in Kg/m^3) to compute mass based on link volume. A value of 0.0 can be used to tell the physics engine to automatically compute density as well.
* **Clean Stage**: When checked, cleans the stage before loading the new MJCF, otherwise loads it on current open stage at position `(0,0,0)`
* **Self Collision**: Enables self collision between adjacent links. It may cause instability if the collision meshes are intersecting at the joint.
* **Create Physics Scene**: Creates a default physics scene on the stage. Because this physics scene is created outside of the scene asset, it will not be loaded into other scenes composed with the robot asset.

**Note:** It is recommended to set Self Collision to false unless you are certain that links on the robot are not self colliding


## Robot Properties

There might be many properties you want to tune on your robot. These properties can be spread across many different Schemas and APIs.

The general steps of getting and setting a parameter are:

1. Find which API is the parameter under. Most common ones can be found in the [Pixar USD API](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/api/pxr_index.html).

2. Get the prim handle that the API is applied to. For example, Articulation and Drive APIs are applied to joints, and MassAPIs are applied to the rigid bodies.

3. Get the handle to the API. From there on, you can Get or Set the attributes associated with that API.

For example, if we want to set the wheel's drive velocity and the actuators' stiffness, we need to find the DriveAPI:


```python
    # get handle to the Drive API for both wheels
    left_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/left_wheel"), "angular")
    right_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/right_wheel"), "angular")

    # Set the velocity drive target in degrees/second
    left_wheel_drive.GetTargetVelocityAttr().Set(150)
    right_wheel_drive.GetTargetVelocityAttr().Set(150)

    # Set the drive damping, which controls the strength of the velocity drive
    left_wheel_drive.GetDampingAttr().Set(15000)
    right_wheel_drive.GetDampingAttr().Set(15000)

    # Set the drive stiffness, which controls the strength of the position drive
    # In this case because we want to do velocity control this should be set to zero
    left_wheel_drive.GetStiffnessAttr().Set(0)
    right_wheel_drive.GetStiffnessAttr().Set(0)
```

Alternatively you can use the [Omniverse Commands Tool](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_kit_commands.html#isaac-sim-command-tool) to change a value in the UI and get the associated Omniverse command that changes the property.

**Note:**
    - The drive stiffness parameter should be set when using position control on a joint drive.
    - The drive damping parameter should be set when using velocity control on a joint drive.
    - A combination of setting stiffness and damping on a drive will result in both targets being applied, this can be useful in position control to reduce vibrations.
