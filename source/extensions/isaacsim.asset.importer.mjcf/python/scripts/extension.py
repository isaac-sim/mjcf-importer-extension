# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc
import os
import weakref
from collections import namedtuple
from pathlib import Path

import carb
import omni.client
import omni.ext
import omni.kit.tool.asset_importer as ai
import omni.ui as ui
from isaacsim.asset.importer.mjcf import _mjcf
from omni.kit.helper.file_utils import asset_types
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.viewport.utility import get_active_viewport
from omni.kit.window.filepicker import FilePickerDialog
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .option_widget import OptionWidget

# from omni.isaac.ui.menu import make_menu_item_description
from .ui_utils import (
    btn_builder,
    cb_builder,
    dropdown_builder,
    float_builder,
    str_builder,
)

EXTENSION_NAME = "MJCF Importer"

import omni.ext
from omni.kit.menu.utils import MenuItemDescription


def make_menu_item_description(ext_id: str, name: str, onclick_fun, action_name: str = "") -> None:
    """Easily replace the onclick_fn with onclick_action when creating a menu description

    Args:
        ext_id (str): The extension you are adding the menu item to.
        name (str): Name of the menu item displayed in UI.
        onclick_fun (Function): The function to run when clicking the menu item.
        action_name (str): name for the action, in case ext_id+name don't make a unique string

    Note:
        ext_id + name + action_name must concatenate to a unique identifier.

    """
    # TODO, fix errors when reloading extensions
    # action_unique = f'{ext_id.replace(" ", "_")}{name.replace(" ", "_")}{action_name.replace(" ", "_")}'
    action_registry = omni.kit.actions.core.get_action_registry()
    action_id = "open_mjcf_importer"
    action_registry.register_action(ext_id, action_id, onclick_fun)
    return MenuItemDescription(name=name, onclick_fn=onclick_fun)


def is_mjcf_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext == ".xml"


def on_filter_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_mjcf_file(item.path)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):

        self._mjcf_interface = _mjcf.acquire_mjcf_interface()
        self._usd_context = omni.usd.get_context()
        # self._window = omni.ui.Window(
        #     EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        # )
        # self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        # self._window.set_visibility_changed_fn(self._on_window)
        # self._window.frame.set_build_fn(self.build_ui)
        # menu_items = [
        #     make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        # ]
        # self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        # add_menu_items(self._menu_items, "Isaac Utils")

        self._models = {}
        result, self._config = omni.kit.commands.execute("MJCFCreateImportConfig")
        self._filepicker = None
        self._last_folder = None
        self._content_browser = None
        self._extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self._imported_robot = None
        self.reset_config()
        self._option_builder = OptionWidget(self._models, self._config)
        self._delegate = MjcfImporterDelegate(
            "Mjcf Importer",
            ["(.*\\.xml$)|(.*\\.XML$)"],
            ["Mjcf Files (*.xml, *.XML)"],
        )
        self._delegate.set_importer(self)
        ai.register_importer(self._delegate)

    def build_new_optons(self):
        self._option_builder.build_options()

    def reset_config(self):
        # Set defaults
        # self._config.set_merge_fixed_joints(False)
        # self._config.set_convex_decomp(False)
        self._config.set_fix_base(False)
        self._config.set_import_inertia_tensor(False)
        self._config.set_distance_scale(1.0)
        self._config.set_density(0.0)
        # self._config.set_default_drive_type(1)
        # self._config.set_default_drive_strength(1e7)
        # self._config.set_default_position_drive_damping(1e5)
        self._config.set_self_collision(False)
        self._config.set_make_default_prim(True)
        self._config.set_create_physics_scene(True)
        self._config.set_import_sites(True)
        self._config.set_isaaclab(False)
        self._config.set_visualize_collision_geoms(False)

    def build_ui(self):
        with ui.VStack(spacing=20, height=0):
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=2, height=0):
                    # cb_builder(
                    #     label="Merge Fixed Joints",
                    #     tooltip="Check this box to skip adding articulation on fixed joints",
                    #     on_clicked_fn=lambda m, config=self._config: config.set_merge_fixed_joints(m),
                    # )
                    self.build_options_frame_left_top()
                with ui.VStack(spacing=2, height=0):
                    self.build_options_frame_right_bottom()

            with ui.VStack(height=0):
                with ui.HStack(spacing=20):
                    btn_builder("Import MJCF", text="Select and Import", on_clicked_fn=self._parse_mjcf)

    def build_options_frame_left_top(self):
        cb_builder(
            "Fix Base Link",
            tooltip="If true, enables the fix base property on the root of the articulation.",
            default_val=False,
            on_clicked_fn=lambda m, config=self._config: config.set_fix_base(m),
        )
        cb_builder(
            "Import Inertia Tensor",
            tooltip="If True, inertia will be loaded from mjcf, if the mjcf does not specify inertia tensor, identity will be used and scaled by the scaling factor. If false physx will compute automatically",
            on_clicked_fn=lambda m, config=self._config: config.set_import_inertia_tensor(m),
        )
        cb_builder(
            "Import Sites",
            tooltip="If True, sites will be imported from mjcf.",
            default_val=True,
            on_clicked_fn=lambda m, config=self._config: config.set_import_sites(m),
        )
        cb_builder(
            "Visualize Collision Geoms",
            tooltip="If True, collision geoms will also be imported as visual geoms",
            default_val=True,
            on_clicked_fn=lambda m, config=self._config: config.set_visualize_collision_geoms(m),
        )
        self._models["scale"] = float_builder(
            "Stage Units Per Meter",
            default_val=1.0,
            tooltip="[1.0 / stage_units] Set the distance units the robot is imported as, default is 1.0 corresponding to m",
        )
        self._models["scale"].add_value_changed_fn(
            lambda m, config=self._config: config.set_distance_scale(m.get_value_as_float())
        )
        # self._models["density"] = float_builder(
        #     "Link Density",
        #     default_val=0.0,
        #     tooltip="[kg/stage_units^3] If a link doesn't have mass, use this density as backup, A density of 0.0 results in the physics engine automatically computing a default density",
        # )
        # self._models["density"].add_value_changed_fn(
        #     lambda m, config=self._config: config.set_density(m.get_value_as_float())
        # )
        # dropdown_builder(
        #     "Joint Drive Type",
        #     items=["None", "Position", "Velocity"],
        #     default_val=1,
        #     on_clicked_fn=lambda i, config=self._config: i,
        #         #config.set_default_drive_type(0 if i == "None" else (1 if i == "Position" else 2)
        #     tooltip="Set the default drive configuration, None: stiffness and damping are zero, Position/Velocity: use default specified below.",
        # )
        # self._models["drive_strength"] = float_builder(
        #     "Joint Drive Strength",
        #     default_val=1e7,
        #     tooltip="Corresponds to stiffness for position or damping for velocity, set to -1 to prevent this value from getting used",
        # )
        # self._models["drive_strength"].add_value_changed_fn(
        #     lambda m, config=self._config: m
        #         # config.set_default_drive_strength(m.get_value_as_float())
        # )
        # self._models["position_drive_damping"] = float_builder(
        #     "Joint Position Drive Damping",
        #     default_val=1e5,
        #     tooltip="If the drive type is set to position, this will be used as a default damping for the drive, set to -1 to prevent this from getting used",
        # )
        # self._models["position_drive_damping"].add_value_changed_fn(
        #     lambda m, config=self._config: m
        #             #config.set_default_position_drive_damping(m.get_value_as_float()
        # )

    def build_options_frame_right_bottom(self):
        self._models["clean_stage"] = cb_builder(
            label="Clean Stage", tooltip="Check this box to load MJCF on a clean stage"
        )
        # cb_builder(
        #     "Convex Decomposition",
        #     tooltip="If true, non-convex meshes will be decomposed into convex collision shapes, if false a convex hull will be used.",
        #     on_clicked_fn=lambda m, config=self._config: config.set_convex_decomp(m),
        # )
        cb_builder(
            "Self Collision",
            tooltip="If true, allows self intersection between links in the robot, can cause instability if collision meshes between links are self intersecting",
            on_clicked_fn=lambda m, config=self._config: config.set_self_collision(m),
        )
        cb_builder(
            "Create Physics Scene",
            tooltip="If true, creates a default physics scene if one does not already exist in the stage",
            default_val=True,
            on_clicked_fn=lambda m, config=self._config: config.set_create_physics_scene(m),
        ),
        cb_builder(
            "Make Default Prim",
            tooltip="If true, makes imported robot the default prim for the stage",
            default_val=True,
            on_clicked_fn=lambda m, config=self._config: config.set_make_default_prim(m),
        )
        cb_builder(
            "Create Instanceable Asset",
            tooltip="If true, creates an instanceable version of the asset. Meshes will be saved in a separate USD file",
            default_val=False,
            on_clicked_fn=lambda m, config=self._config: config.set_make_instanceable(m),
        )
        self._models["instanceable_usd_path"] = str_builder(
            "Instanceable USD Path",
            tooltip="USD file to store instanceable meshes in",
            default_val="./instanceable_meshes.usd",
            use_folder_picker=True,
            folder_dialog_title="Select Output File",
            folder_button_title="Select File",
        )
        self._models["instanceable_usd_path"].add_value_changed_fn(
            lambda m, config=self._config: config.set_instanceable_usd_path(m.get_value_as_string())
        )

    # def _menu_callback(self):
    #     self._window.visible = not self._window.visible

    # def _on_window(self, visible):
    #     if self._window.visible:
    #         self._events = self._usd_context.get_stage_event_stream()
    #     else:
    #         self._events = None
    #         self._stage_event_sub = None

    def _refresh_filebrowser(self):
        parent = None
        selection_name = None
        if len(self._filebrowser.get_selections()):
            parent = self._filebrowser.get_selections()[0].parent
            selection_name = self._filebrowser.get_selections()[0].name

        self._filebrowser.refresh_ui(parent)
        if selection_name:
            selection = [child for child in parent.children.values() if child.name == selection_name]
            if len(selection):
                self._filebrowser.select_and_center(selection[0])

    def _parse_mjcf(self):
        self._filepicker = FilePickerDialog(
            "Import MJCF",
            allow_multi_selection=False,
            apply_button_label="Import",
            click_apply_handler=lambda filename, path, c=weakref.proxy(self): c._select_picked_file_callback(
                self._filepicker, filename, path
            ),
            click_cancel_handler=lambda a, b, c=weakref.proxy(self): c._filepicker.hide(),
            item_filter_fn=on_filter_item,
            enable_versioning_pane=True,
        )
        if self._last_folder:
            self._filepicker.set_current_directory(self._last_folder)
            self._filepicker.navigate_to(self._last_folder)
            self._filepicker.refresh_current_directory()
        self._filepicker.toggle_bookmark_from_path("Built In MJCF Files", (self._extension_path + "/data/mjcf"), True)
        self._filepicker.show()

    def _load_robot(self, path=None, **kargs):
        export_folder = (
            self._models["instanceable_usd_path"].get_value_as_string() if self._models["instanceable_usd_path"] else ""
        )
        if export_folder == "Same as Imported Model(Default)":
            export_folder = ""
        if path:
            base_path = path[: path.rfind("/")]
            basename = path[path.rfind("/") + 1 :]
            basename = basename[: basename.rfind(".")]
            if path.rfind("/") < 0:
                base_path = path[: path.rfind("\\")]
                basename = path[path.rfind("\\") + 1]

            # sanitize basename
            if basename[0].isdigit():
                basename = "_" + basename

            if export_folder:
                base_path = export_folder

            full_path = os.path.abspath(os.path.join(self.root_path, self.filename))
            dest_path = "{}/{}/{}.usd".format(base_path, basename, basename)
            if not self._models.get("import_as_reference", False):
                dest_path = ""
            current_stage = omni.usd.get_context().get_stage()
            prim_path = omni.usd.get_stage_next_free_path(current_stage, "/" + basename, False)

            omni.kit.commands.execute(
                "MJCFCreateAsset",
                mjcf_path=full_path,
                import_config=self._config,
                prim_path=prim_path,
                dest_path=dest_path,
            )
            stage = Usd.Stage.Open(dest_path)
            prim_name = str(stage.GetDefaultPrim().GetName())

            def add_reference_to_stage():
                current_stage = omni.usd.get_context().get_stage()
                if current_stage:
                    prim_path = omni.usd.get_stage_next_free_path(
                        current_stage, str(current_stage.GetDefaultPrim().GetPath()) + "/" + prim_name, False
                    )
                    robot_prim = current_stage.OverridePrim(prim_path)
                    if "anon:" in current_stage.GetRootLayer().identifier:
                        robot_prim.GetReferences().AddReference(dest_path)
                    else:
                        robot_prim.GetReferences().AddReference(
                            omni.client.make_relative_url(current_stage.GetRootLayer().identifier, dest_path)
                        )
                    if self._config.create_physics_scene:
                        UsdPhysics.Scene.Define(current_stage, Sdf.Path("/physicsScene"))

            async def import_with_clean_stage():
                await omni.usd.get_context().new_stage_async()
                await omni.kit.app.get_app().next_update_async()
                current_stage = omni.usd.get_context().get_stage()
                UsdGeom.SetStageUpAxis(current_stage, UsdGeom.Tokens.z)
                UsdGeom.SetStageMetersPerUnit(stage, 1)
                add_reference_to_stage()
                await omni.kit.app.get_app().next_update_async()

            if self._models["clean_stage"].get_value_as_bool():
                asyncio.ensure_future(import_with_clean_stage())
            else:
                upAxis = UsdGeom.GetStageUpAxis(current_stage)
                if upAxis == "Y":
                    carb.log_error("The stage Up-Axis must be Z to use the MJCF importer")
                add_reference_to_stage()
            return prim_path

    def _select_picked_file_callback(self, dialog: FilePickerDialog, filename=None, path=None):
        if not path.startswith("omniverse://"):
            self.root_path = path
            self.filename = filename
            if path and filename:
                self._last_folder = path
                prim_path = self._load_robot(path + "/" + filename)
                task = asyncio.ensure_future(omni.kit.app.get_app().next_update_async())
                asyncio.ensure_future(task)
                viewport_api = get_active_viewport()
                stage = viewport_api.stage
                mpu = UsdGeom.GetStageMetersPerUnit(stage)
                cam_path = viewport_api.camera_path
                omni.kit.commands.execute(
                    "FramePrimsCommand",
                    prim_to_move=cam_path,
                    prims_to_frame=[prim_path],
                    aspect_ratio=1,
                    zoom=0.05 * (self._models["scale"].get_value_as_float()),
                )
            else:
                carb.log_error("path and filename not specified")
        else:
            carb.log_error("Only Local Paths supported")
        dialog.hide()

    def on_shutdown(self):
        _mjcf.release_mjcf_interface(self._mjcf_interface)
        if self._filepicker:
            self._filepicker.toggle_bookmark_from_path(
                "Built In MJCF Files", (self._extension_path + "/data/mjcf"), False
            )
            self._filepicker.destroy()
            self._filepicker = None

        # remove_menu_items(self._menu_items, "Isaac Utils")
        # if self._window:
        #     self._window = None

        self._delegate.destroy()
        ai.remove_importer(self._delegate)
        gc.collect()


class MjcfImporterDelegate(ai.AbstractImporterDelegate):
    """
    Mjcf importer delegate implementation for Asset Importer AbstractImporterDelegate.
    """

    def __init__(self, name, filters, descriptions):
        super().__init__()
        self._name = name
        self._filters = filters
        self._descriptions = descriptions
        self._importer = None
        # register the mjcf icon to asset types
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        icon_path = Path(ext_path).joinpath("icons").absolute()
        AssetTypeDef = namedtuple("AssetTypeDef", "glyph thumbnail matching_exts")
        known_asset_types = asset_types.known_asset_types()
        known_asset_types["mjcf"] = AssetTypeDef(
            f"{icon_path}/icoFileMJCF.png",
            f"{icon_path}/icoFileMJCF.png",
            [".xml", ".XML"],
        )

    def set_importer(self, importer):
        self._importer = importer

    def show_destination_frame(self):
        return False

    def destroy(self):
        self._importer = None

    def _on_import_complete(self, file_paths):
        pass

    @property
    def name(self):
        return self._name

    @property
    def filter_regexes(self):
        return self._filters

    @property
    def filter_descriptions(self):
        return self._descriptions

    def build_options(self, paths):
        # self._importer.reset_config()
        # self._importer.build_options_frame_left_top()
        # self._importer.build_options_frame_right_bottom()
        self._importer.build_new_optons()

    async def convert_assets(self, paths, **kargs):
        if not paths:
            post_notification(
                "No file selected",
                "Please select a file to import",
                NotificationStatus.ERROR,
            )
            return None
        for path in paths:
            self._importer.root_path = os.path.dirname(path)
            self._importer.filename = os.path.basename(path)
            prim_path = self._importer._load_robot(path, **kargs)
            task = asyncio.ensure_future(omni.kit.app.get_app().next_update_async())
            asyncio.ensure_future(task)
            viewport_api = get_active_viewport()
            stage = viewport_api.stage
            mpu = UsdGeom.GetStageMetersPerUnit(stage)
            cam_path = viewport_api.camera_path
            omni.kit.commands.execute(
                "FramePrimsCommand",
                prim_to_move=cam_path,
                prims_to_frame=[prim_path],
                aspect_ratio=1,
                zoom=0.05,
            )
        # Don't need to return dest path here, _load_robot do the insertion to stage
        return {}
