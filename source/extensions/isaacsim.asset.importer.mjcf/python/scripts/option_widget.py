# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.ui as ui

from .style import get_option_style
from .ui_utils import add_folder_picker_icon, format_tt


def checkbox_builder(label="", type="checkbox", default_val=False, tooltip="", on_clicked_fn=None):
    """Creates a Stylized Checkbox

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "checkbox".
        default_val (bool, optional): Checked is True, Unchecked is False. Defaults to False.
        tooltip (str, optional): Tooltip to display over the Label. Defaults to "".
        on_clicked_fn (Callable, optional): Call-back function when clicked. Defaults to None.

    Returns:
        ui.SimpleBoolModel: model
    """

    with ui.HStack():
        check_box = ui.CheckBox(width=10, height=0)
        ui.Spacer(width=8)
        check_box.model.set_value(default_val)

        def on_click(value_model):
            on_clicked_fn(value_model.get_value_as_bool())

        if on_clicked_fn:
            check_box.model.add_value_changed_fn(on_click)
        ui.Label(label, width=0, height=0, tooltip=tooltip)
        return check_box.model


def float_field_builder(label="", default_val=0, tooltip="", format="%.2f"):
    """Creates a Stylized Floatfield Widget

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        default_val (int, optional): Default Value of UI element. Defaults to 0.
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".

    Returns:
        AbstractValueModel: model
    """
    with ui.HStack(spacing=10, style=get_option_style()):
        ui.Label(label, width=ui.Fraction(0.5), alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        with ui.ZStack():
            float_field = ui.FloatDrag(
                name="FloatDrag",
                width=ui.Fraction(0),
                height=0,
                alignment=ui.Alignment.LEFT,
                format=format,
                min=0,
            ).model
            float_field.set_value(default_val)
            with ui.HStack():
                ui.Spacer()
                ui.Label("Kg/m", name="density", alignment=ui.Alignment.RIGHT_CENTER)
                ui.Label("3", name="exponent", alignment=ui.Alignment.RIGHT_TOP, width=0)
                ui.Spacer(width=1)
        return float_field


def string_filed_builder(
    default_val=" ",
    tooltip="",
    read_only=False,
    item_filter_fn=None,
    folder_dialog_title="Select Output Folder",
    folder_button_title="Select Folder",
):
    """Creates a Stylized Stringfield Widget

    Args:
        default_val (str, optional): Text to initialize in Stringfield. Defaults to " ".
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".
        read_only (bool, optional): Prevents editing. Defaults to False.
        item_filter_fn (Callable, optional): filter function to pass to the FilePicker
        bookmark_label (str, optional): bookmark label to pass to the FilePicker
        bookmark_path (str, optional): bookmark path to pass to the FilePicker
    Returns:
        AbstractValueModel: model of Stringfield
    """
    with ui.HStack():
        str_field = ui.StringField(
            name="StringField",
            tooltip=format_tt(tooltip),
            width=ui.Fraction(1),
            height=0,
            alignment=ui.Alignment.LEFT_CENTER,
            read_only=read_only,
        )
        str_field.enabled = False
        str_field.model.set_value(default_val)

        def update_field(filename, path):
            if filename == "":
                val = path
            elif filename[0] != "/" and path[-1] != "/":
                val = path + "/" + filename
            elif filename[0] == "/" and path[-1] == "/":
                val = path + filename[1:]
            else:
                val = path + filename
            str_field.model.set_value(val)

        ui.Spacer(width=4)
        file_pick_fn = add_folder_picker_icon(
            update_field, item_filter_fn, dialog_title=folder_dialog_title, button_title=folder_button_title, size=16
        )
        ui.Spacer(width=2)
        str_field.set_mouse_pressed_fn(lambda a, b, c, d: file_pick_fn())
        return str_field.model


def option_header(collapsed, title):
    with ui.HStack(height=22):
        ui.Spacer(width=4)
        with ui.VStack(width=10):
            ui.Spacer()
            if collapsed:
                triangle = ui.Triangle(height=7, width=5)
                triangle.alignment = ui.Alignment.RIGHT_CENTER
            else:
                triangle = ui.Triangle(height=5, width=7)
                triangle.alignment = ui.Alignment.CENTER_BOTTOM
            ui.Spacer()
        ui.Spacer(width=4)
        ui.Label(title, name="collapsable_header", width=0)
        ui.Spacer(width=3)
        ui.Line()


def option_frame(title, build_content_fn, collapse_fn=None):
    with ui.CollapsableFrame(
        title, name="option", height=0, collapsed=False, build_header_fn=option_header, collapsed_changed_fn=collapse_fn
    ):
        with ui.HStack():
            ui.Spacer(width=2)
            build_content_fn()
            ui.Spacer(width=2)


class OptionWidget:
    def __init__(self, models, config):
        self._models = models
        self._config = config

    @property
    def models(self):
        return self._models

    @property
    def config(self):
        return self._config

    def build_options(self):
        with ui.VStack(style=get_option_style()):
            self._build_model_frame()
            self._build_links_frame()
            self._build_colliders_frame()

    def _build_model_frame(self):
        def build_model_content():
            with ui.VStack(spacing=4):
                with ui.HStack(height=26):
                    self._import_collection = ui.RadioCollection()
                    with ui.VStack(width=0, alignment=ui.Alignment.LEFT_CENTER):
                        ui.Spacer()
                        ui.RadioButton(
                            width=20,
                            height=20,
                            radio_collection=self._import_collection,
                            alignment=ui.Alignment.LEFT_CENTER,
                        )
                        ui.Spacer()
                    ui.Spacer(width=4)
                    ui.Label("Create in Stage", width=90)
                    ui.Spacer(width=10)
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.RadioButton(
                            width=20,
                            height=20,
                            radio_collection=self._import_collection,
                            alignment=ui.Alignment.LEFT_CENTER,
                        )
                        ui.Spacer()
                    ui.Spacer(width=4)
                    ui.Label("Referenced Model")
                    ui.Spacer(width=50)
                    self._import_collection.model.set_value(1)
                    self._import_collection.model.add_value_changed_fn(lambda m: self._update_import_option(m))
                    self._models["import_as_reference"] = True
                self._add_as_reference_frame = ui.VStack()
                with self._add_as_reference_frame:
                    ui.Label("USD Output")
                    self._models["instanceable_usd_path"] = string_filed_builder(
                        tooltip="USD file to store instanceable meshes in",
                        default_val="Same as Imported Model(Default)",
                        folder_dialog_title="Select Output File",
                        folder_button_title="Select File",
                        read_only=True,
                    )
                self._add_to_stage_frame = ui.VStack()
                with self._add_to_stage_frame:
                    # TODO: when change this to False, will raise a 'not found default prim' error in _load_robot
                    checkbox_builder(
                        "Set as Default Prim",
                        tooltip="If true, makes imported robot the default prim for the stage",
                        default_val=self._config.make_default_prim,
                        on_clicked_fn=lambda m, config=self._config: config.set_make_default_prim(m),
                    )

                    self._models["clean_stage"] = checkbox_builder(
                        "Clear Stage on Import",
                        tooltip="Check this box to load URDF on a clean stage",
                        default_val=False,
                    )
                self._add_to_stage_frame.visible = False
                checkbox_builder(
                    "Import Sites",
                    tooltip="If True, sites will be imported from mjcf.",
                    default_val=True,
                    on_clicked_fn=lambda m, config=self._config: config.set_import_sites(m),
                )
                checkbox_builder(
                    "Create for Isaac Lab",
                    tooltip="If True, the structure of the USD will be Isaac Lab compatible.",
                    default_val=False,
                    on_clicked_fn=lambda m, config=self._config: config.set_isaaclab(m),
                )

        option_frame("Model", build_model_content)

    def _build_links_frame(self):
        def build_links_content():
            with ui.VStack(spacing=4):
                with ui.HStack(height=24):
                    ui.Spacer(width=0)
                    self._base_collection = ui.RadioCollection()
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.RadioButton(width=20, height=20, radio_collection=self._base_collection)
                        ui.Spacer()
                    ui.Spacer(width=4)
                    ui.Label("Moveable Base", width=90)
                    ui.Spacer(width=10)
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.RadioButton(width=20, height=20, radio_collection=self._base_collection)
                        ui.Spacer()
                    ui.Spacer(width=4)
                    ui.Label("Static Base")
                    index = 1 if self._config.fix_base else 0
                    self._base_collection.model.set_value(index)
                    self._base_collection.model.add_value_changed_fn(lambda m: self._update_fix_base(m))
                self._models["density"] = float_field_builder(
                    "Default Density",
                    default_val=self._config.density,
                    tooltip="[kg/stage_units^3] If a link doesn't have mass, use this density as backup, A density of 0.0 results in the physics engine automatically computing a default density",
                )
                self._models["density"].add_value_changed_fn(
                    lambda m, config=self._config: config.set_density(m.get_value_as_float())
                )

        option_frame("Links", build_links_content)

    def _update_fix_base(self, model):
        value = model.get_value_as_bool()
        self._config.set_fix_base(value)
        self._config.set_visualize_collision_geoms(False)

    def _build_colliders_frame(self):
        def build_colliders_content():
            with ui.VStack(spacing=4):
                checkbox_builder(
                    "Visualize Collision Geometry",
                    tooltip="If True, collision geoms will also be imported as visual geoms",
                    default_val=False,
                    on_clicked_fn=lambda m, config=self._config: config.set_visualize_collision_geoms(m),
                )

                checkbox_builder(
                    "Self Collision",
                    tooltip="If true, allows self intersection between links in the robot, can cause instability if collision meshes between links are self intersecting",
                    default_val=self._config.self_collision,
                    on_clicked_fn=lambda m, config=self._config: config.set_self_collision(m),
                )

        option_frame("Colliders", build_colliders_content)

    def _update_import_option(self, model):
        value = bool(model.get_value_as_int() == 1)
        self._add_as_reference_frame.visible = value
        self._add_to_stage_frame.visible = not value
        self._models["import_as_reference"] = value
