# Usage

To enable this extension, go to the Extension Manager menu and enable omni.importer.mjcf extension.


# High Level Code Overview

## Python
The `MJCF Importer` extension sets attributes of `ImportConfig` on initialization,
along with the UI giving the user options to change certain attributes, such as `set_fix_base`.
The complete list of configs can be found in `bindings/BindingsMjcfPython.cpp`.


In `python/scripts/commands.py`, `MJCFCreateAsset` defines a command that takes
in `ImportConfig` and file path/usd path related to the desired MJCF file to import;
it then calls `self._mjcf_interface.create_asset_mjcf`, which binds to
the C++ function `createAssetFromMJCF` in `plugins/Mjcf.cpp`.


## C++

`plugins/Mjcf.cpp` contains the `createAssetFromMJCF` function, which is the entry
point to parsing the MJCF file and converting it to a USD file. In this function, it
initializes the `MJCFImporter` class from `plugins/MJCFImporter.cpp` which parses the MJCF file,
sets up a UsdStage in accordance with the import config settings, creates the parsed entities
to the stage via `MJCFImporter::AddPhysicsEntities`, and saves the stage if specified in the config.

Upon initialization of the `MJFImporter` class, it parses the given MJCF file by mainly utilziing
functions from `plugins/MjcfParser.cpp` as follows:
- Initializes various buffers to contain bodies, actuators, tendons, etc.
- Calls `LoadFile`, which parses the xml file using tinyxml2 and returns the root element.
- Calls `LoadInclude`, which parses any xml file referenced using the `<include filename='...'>` tag
- Calls `LoadGlobals`, which performs the majority of the parsing by saving all bodies, actuators,
tendons, contact pairs, etc. and their associated settings into classes (defined in `plugins/MjcfTypes.h`)
to be converted into USD assets later on. Details of this function are described in a seperate section below.
- Calls `populateBodyLookup` recursively to go through all bodies in the kinematic tree and populates `nameToBody`,
which maps from the body name to its `MJCFBody`. It also records all the geometries that participate in collision in `geomNameToIdx` and `collisionGeoms`, which are used to populate a contact graph later on.
- Calls `computeKinematicHierarchy`, which runs breadth-first search on the kinematic tree to determine the depth
of each body on the kinematic tree. For instance, the root body has a depth of 0 and its children have a depth of 1, etc.
This information is used to determine the parent-child relationship of the bodies, which is used later on when importing
tendons to USD.
- Calls `createContactGraph` to populate a graph where each node represents a body that participates in collisions and its
neighboring nodes are the bodies that it can collide with.


`LoadGlobals` from `plugins/MjcfParser.cpp`:
- Calls `LoadCompiler`, which saves settings defined in the `<compiler>` tag into the `MJCFCompiler` class.
`MJCFCompiler` contains attributes such as the unit of measurement of angles (rad/deg), the mesh directory
path, the Euler rotation sequence (xyz/zyx/etc.).
- Parses the `<default>` tags, which calls `LoadDefault` and saves settings for bodies, actuators, tendons,
etc into an `MJCFClass` for each default tag. Note that `LoadDefault` is recursively called to deal with
nested `<default>` tags.
- Calls `LoadAssets`, which saves data regarding meshes and textures into the `MJCFMesh` and `MJCFTexture` classes respectively.
- Finds the `<worldbody>` tag, which defines the origin of the world frame within which the rest of the kinematic tree is defined. From there, it calls `LoadInclude` to load any included file and then calls `LoadBody` recursively
to save data regarding the kinematic tree into the `MJCFBody` class. It also calls `LoadGeom` and `LoadSite`. Note that the `MJCFBody` class contains the following attributes: name, pose, inertial, a list of geometries (`MJCFGeom`), a list of joints that attaches to the body (`MJCFJoint`), and a list of child bodies.
- Calls `LoadActuator` for all the `<actuator>` tags. For each actuator, there is an assoicated joint, which is saved in the `jointToActuatorIdx` map.
- Calls `LoadTendon` for all the `<tendon>` tags, which saves data regarding each tendon into `MJCFTendon` classes. For each fixed tendon, `MJCFTendon` contains a list of joints that the tendon is attached to. For each spatial tendon, `MJCFTendon` contains a list of spatial attachements, pulleys, and branches.
- Calls `LoadContact` to parse contact pairs and contact exclusions into the `MJCFContact` class.


`MJCFImporter::AddPhysicsEntities` adds all the parsed entities to the stage by mainly utilizing functions
from `plugins/MjcfUsd.cpp` as follows:
- Calls `setStageMetadata`, which sets up the UsdPhysicsScene
- Calls `createRoot` which defines the robot's root USD prim. Will also make this prim the default prim if
makeDefaultPrim is set to true in the import config.
- Handles making the imported USD instanceable if desired in the import config. For more information regarding instanceable assets, please visit https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_instanceable_assets.html
- For each body, calls `CreatePhysicsBodyAndJoint` recursively, which imports the kinematic tree onto the USD stage.
- Calls `addWorldGeomsAndSites`, which creates dummy links to place the sites/geoms defined in the world body
- Calls `AddContactFilters`, which adds collisions between the prims in accordance with the contact graph.
- Calls `AddTendons` to add all the fixed and spatial tendons.
