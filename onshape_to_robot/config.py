"""Load configuration from config.json."""
from __future__ import annotations

import dataclasses
import os
import sys
from pathlib import Path
from typing import (
    Any,
    Dict,
    Optional,
    Union,
)

import commentjson as json
from colorama import (
    Fore,
    Style,
)
from easydict import EasyDict

from .material_tags import load_material_tags


def load_config(path: Optional[Union[Path, str]] = None) -> EasyDict[str, Any]:
    """Load configuration from the specified path or args."""
    return Config.from_path(Path(path)) if path else Config.from_argv()


@dataclasses.dataclass()
class Config:
    """Load configuration from a config.json and parse it."""

    params: Dict[str, Any] = dataclasses.field(default_factory=dict)

    def get(self, name, default=None, hasDefault=False, valuesList=None):
        """Get a value from the config with defaults."""
        hasDefault = hasDefault or (default is not None)

        if name in self.params:
            value = self.params[name]
            if valuesList is not None and value not in valuesList:
                print(
                    Fore.RED
                    + "ERROR: Value for "
                    + name
                    + " should be one of: "
                    + (",".join(valuesList))
                    + Style.RESET_ALL
                )
                sys.exit(1)
            return value
        else:
            if hasDefault:
                return default
            else:
                print(
                    Fore.RED
                    + 'ERROR: missing key "'
                    + name
                    + '" in config'
                    + Style.RESET_ALL
                )
                sys.exit(1)

    @classmethod
    def from_argv(cls) -> EasyDict[str, Any]:
        """Load config with the path specified in argv."""
        if len(sys.argv) <= 1:
            print(
                Fore.RED
                + "ERROR: usage: onshape-to-robot {robot_directory}"
                + Style.RESET_ALL
            )
            print("Read documentation at https://onshape-to-robot.readthedocs.io/")
            sys.exit(1)
        return cls.from_path(path=sys.argv[1])

    @classmethod
    def from_path(cls, path) -> EasyDict[str, Any]:
        """Load configuration from the specified path."""
        path = Path(path)
        config_path = path / "config.json"

        if not os.path.exists(config_path):
            print(
                f"{Fore.RED}ERROR: Config file {config_path} not found."
                + Style.RESET_ALL
            )
            sys.exit(1)
        with open(config_path, "r", encoding="utf8") as stream:
            loaded_config = Config(json.load(stream))

        settings = [
            ("documentsUrl", ""),
            ("documentId", ""),
            ("versionId", ""),
            ("workspaceId", ""),
            ("drawFrames", False),
            ("drawCollisions", False),
            ("assemblyName", False),
            ("outputFormat", "urdf"),
            ("useFixedLinks", False),
            ("configuration", "default"),
            ("ignoreLimits", False),
            # Using OpenSCAD for simplified geometry
            ("useScads", True),
            ("pureShapeDilatation", 0.0),
            # Dynamics
            ("jointMaxEffort", 1),
            ("jointMaxVelocity", 20),
            ("noDynamics", False),
            # Ignore list
            ("ignore", []),
            ("ignoreRegex", []),
            ("whitelist", None, True),
            # Color override
            ("color", None, True),
            # STLs merge and simplification
            ("mergeSTLs", "no", False, ["no", "visual", "collision", "all"]),
            ("maxSTLSize", 3),
            ("simplifySTLs", "no", False, ["no", "visual", "collision", "all"]),
            # Post-import commands to execute
            ("postImportCommands", []),
            # Add collisions=true configuration on parts
            ("useCollisionsConfigurations", True),
            # Use Onshape materials as tags for collision objects.
            ("materialTagsOnly", False),
            # ROS support
            ("packageName", ""),
            ("packageType", "none"),
            ("addDummyBaseLink", False),
            ("robotName", "onshape"),
        ]

        params = {args[0]: loaded_config.get(*args) for args in settings}

        params["materialTags"] = load_material_tags(
            loaded_config.get("materialTags", [])
        )
        params["outputDirectory"] = path
        params["configPath"] = config_path
        params["dynamicsOverride"] = {}

        # additional XML code to insert
        if params["outputFormat"] == "urdf":
            additional_file_name = loaded_config.get("additionalUrdfFile", "")
        else:
            additional_file_name = loaded_config.get("additionalSdfFile", "")

        if additional_file_name == "":
            params["additionalXML"] = ""
        else:
            with open(path / additional_file_name, "r", encoding="utf-8") as stream:
                params["additionalXML"] = stream.read()

        # Creating dynamics override array
        tmp = loaded_config.get("dynamics", {})
        for key in tmp:
            if tmp[key] == "fixed":
                params["dynamicsOverride"][key.lower()] = {
                    "com": [0, 0, 0],
                    "mass": 0,
                    "inertia": [0, 0, 0, 0, 0, 0, 0, 0, 0],
                }
            else:
                params["dynamicsOverride"][key.lower()] = tmp[key]

        # Output directory, making it if it doesn't exists
        try:
            os.makedirs(params["outputDirectory"])
        except OSError:
            pass

        # Validation
        ALLOWED_PACKAGE_TYPES = {"none", "ament", "catkin"}
        if params["packageType"] not in ALLOWED_PACKAGE_TYPES:
            raise ValueError(
                "packageType '{}' must be one of: {}".format(
                    params["packageType"], list(ALLOWED_PACKAGE_TYPES)
                )
            )

        # Checking that OpenSCAD is present
        if params["useScads"]:
            print(Style.BRIGHT + "* Checking OpenSCAD presence..." + Style.RESET_ALL)
            if os.system("openscad -v 2> /dev/null") != 0:
                print(
                    Fore.RED
                    + "Can't run openscad -v, disabling OpenSCAD support"
                    + Style.RESET_ALL
                )
                print(
                    Fore.BLUE + "TIP: consider installing openscad:" + Style.RESET_ALL
                )
                print(
                    Fore.BLUE
                    + "sudo add-apt-repository ppa:openscad/releases"
                    + Style.RESET_ALL
                )
                print(Fore.BLUE + "sudo apt-get update" + Style.RESET_ALL)
                print(Fore.BLUE + "sudo apt-get install openscad" + Style.RESET_ALL)
                params["useScads"] = False

        # Checking that MeshLab is present
        if params["simplifySTLs"]:
            print(Style.BRIGHT + "* Checking MeshLab presence..." + Style.RESET_ALL)
            if not os.path.exists("/usr/bin/meshlabserver") != 0:
                print(
                    Fore.RED
                    + "No /usr/bin/meshlabserver, disabling STL simplification support"
                    + Style.RESET_ALL
                )
                print(Fore.BLUE + "TIP: consider installing meshlab:" + Style.RESET_ALL)
                print(Fore.BLUE + "sudo apt-get install meshlab" + Style.RESET_ALL)
                params["simplifySTLs"] = False

        # Checking that versionId and workspaceId are not set on same time
        if params["versionId"] != "" and params["workspaceId"] != "":
            print(
                Fore.RED
                + "You can't specify workspaceId AND versionId"
                + Style.RESET_ALL
            )

        return EasyDict(params)
