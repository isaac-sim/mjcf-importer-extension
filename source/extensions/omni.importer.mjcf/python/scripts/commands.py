# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

import omni.client
import omni.kit.commands

# import omni.kit.utils
from omni.client._omniclient import Result
from omni.importer.mjcf import _mjcf
from pxr import Usd


class MJCFCreateImportConfig(omni.kit.commands.Command):
    """
    Returns an ImportConfig object that can be used while parsing and importing.
    Should be used with the `MJCFCreateAsset` command

    Returns:
        :obj:`omni.importer.mjcf._mjcf.ImportConfig`: Parsed MJCF stored in an internal structure.

    """

    def __init__(self) -> None:
        pass

    def do(self) -> _mjcf.ImportConfig:
        return _mjcf.ImportConfig()

    def undo(self) -> None:
        pass


class MJCFCreateAsset(omni.kit.commands.Command):
    """
    This command parses and imports a given mjcf file.

    Args:
        arg0 (:obj:`str`): The absolute path the mjcf file

        arg1 (:obj:`omni.importer.mjcf._mjcf.ImportConfig`): Import configuration

        arg2 (:obj:`str`): Path to the robot on the USD stage

        arg3 (:obj:`str`): destination path for robot usd. Default is "" which will load the robot in-memory on the open stage.

    """

    def __init__(
        self, mjcf_path: str = "", import_config=_mjcf.ImportConfig(), prim_path: str = "", dest_path: str = ""
    ) -> None:
        self.prim_path = prim_path
        self.dest_path = dest_path
        self._mjcf_path = mjcf_path
        self._root_path, self._filename = os.path.split(os.path.abspath(self._mjcf_path))
        self._import_config = import_config
        self._mjcf_interface = _mjcf.acquire_mjcf_interface()
        pass

    def do(self) -> str:
        # if self.prim_path:
        #     self.prim_path = self.prim_path.replace(
        #         "\\", "/"
        #     )  # Omni client works with both slashes cross platform, making it standard to make it easier later on

        if self.dest_path:
            self.dest_path = self.dest_path.replace(
                "\\", "/"
            )  # Omni client works with both slashes cross platform, making it standard to make it easier later on
            result = omni.client.read_file(self.dest_path)
            if result[0] != Result.OK:
                stage = Usd.Stage.CreateNew(self.dest_path)
                stage.Save()

        return self._mjcf_interface.create_asset_mjcf(
            self._mjcf_path, self.prim_path, self._import_config, self.dest_path
        )

    def undo(self) -> None:
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
