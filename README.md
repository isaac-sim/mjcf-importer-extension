# MJCF Importer

The MJCF Importer Extension is used to import MuJoCo representations of scenes.
MuJoCo Modeling XML File (MJCF), is an XML format for representing a scene in the MuJoCo simulator.

## Getting Started

1. Clone the GitHub repo to your local machine.
2. Open a command prompt and navigate to the root of your cloned repo.
3. Run `build.bat` to bootstrap your dev environment and build the example extensions.
4. Run `_build\{platform}\release\omni.importer.mjcf.app.bat` to start the Kit application.
5. From the menu, select `File->Import` and select your MJCF file to import.

This extension is enabled by default. If it is ever disabled, it can be re-enabled from the Extension Manager
by searching for `isaacsim.asset.importer.mjcf`.

**Note:** On Linux, replace `.bat` with `.sh` in the instructions above.

For additional information, check [MJCF Importer Extension Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_asset_importer_mjcf.html).