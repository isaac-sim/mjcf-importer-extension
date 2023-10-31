MJCF Importer Extension [omni.importer.mjcf]
############################################

MJCF Import Commands
====================
The following commands can be used to simplify the import process.
Below is a sample demonstrating how to import the Ant MJCF included with this extension

.. code-block:: python
    :linenos:

    import omni.kit.commands
    from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools

    # setting up import configuration:
    status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
    import_config.set_fix_base(True)
    import_config.set_import_inertia_tensor(True)

    # Get path to extension data:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.importer.mjcf")
    extension_path = ext_manager.get_extension_path(ext_id)

    # import MJCF
    omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=extension_path + "/data/mjcf/nv_ant.xml",
        import_config=import_config,
        prim_path="/ant"
    )

    # get stage handle
    stage = omni.usd.get_context().get_stage()

    # enable physics
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    # set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)


.. automodule:: omni.importer.mjcf.scripts.commands
    :members:
    :undoc-members:
    :exclude-members: do, undo

.. automodule:: omni.importer.mjcf._mjcf

.. autoclass:: omni.importer.mjcf._mjcf.Mjcf
    :members:
    :undoc-members:
    :no-show-inheritance:

.. autoclass:: omni.importer.mjcf._mjcf.ImportConfig
    :members:
    :undoc-members:
    :no-show-inheritance:
