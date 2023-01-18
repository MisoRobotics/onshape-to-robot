import functools
import hashlib
import os
import re
from pathlib import Path
from sys import exit
from typing import (
    Any,
    Dict,
)

import commentjson as json
import numpy as np
from colorama import (
    Fore,
    Style,
)
from onshape_client.oas.models.bt_part_metadata_info import BTPartMetadataInfo

from . import (
    csg,
    ros_package,
)
from .material_tags import (
    MaterialTag,
    get_material_tag,
)
from .robot_description import (
    RobotSDF,
    RobotURDF,
)


def main():
    # Loading configuration, collecting occurrences and building robot tree
    from .load_robot import (
        client,
        config,
        frames,
        getOccurrence,
        occurrenceNameById,
        occurrences,
        tree,
    )

    # Creating robot for output
    if config["outputFormat"] == "urdf":
        robot = RobotURDF(config["robotName"])
    elif config["outputFormat"] == "sdf":
        robot = RobotSDF(config["robotName"])
    else:
        print(
            Fore.RED
            + "ERROR: Unknown output format: "
            + config["outputFormat"]
            + " (supported are urdf and sdf)"
            + Style.RESET_ALL
        )
        exit()
    robot.drawCollisions = config["drawCollisions"]
    robot.jointMaxEffort = config["jointMaxEffort"]
    robot.mergeSTLs = config["mergeSTLs"]
    robot.maxSTLSize = config["maxSTLSize"]
    robot.simplifySTLs = config["simplifySTLs"]
    robot.jointMaxVelocity = config["jointMaxVelocity"]
    robot.noDynamics = config["noDynamics"]
    robot.packageName = config["packageName"]
    robot.packageType = config["packageType"]
    robot.addDummyBaseLink = config["addDummyBaseLink"]
    robot.robotName = config["robotName"]
    robot.additionalXML = config["additionalXML"]
    robot.useFixedLinks = config["useFixedLinks"]
    robot.output_dir = Path(config["outputDirectory"])

    use_material_tags = len(config["materialTags"]) > 0
    robot.use_material_tags = use_material_tags

    material_tags_only = config["materialTagsOnly"]

    material_tags_by_part_id: Dict[str, MaterialTag] = {}
    ignore_regex = [re.compile(pattern) for pattern in config["ignoreRegex"]]

    def partIsIgnore(name):
        if config["whitelist"] is None:
            return name in config["ignore"] or any(
                [p.search(name) for p in ignore_regex]
            )
        else:
            return name not in config["whitelist"]

    def renamePart(name):
        """Remove special characters from part names."""
        return name.replace('"', "in").replace("'", "ft")

    @functools.lru_cache()
    def _get_parts(
        did: str, mid: str, eid: str, configuration: str = ""
    ) -> Dict[str, BTPartMetadataInfo]:
        parts = client._api.onshape_client.parts_api.get_parts_wmve(
            did, "m", mid, eid, configuration=configuration
        )
        return {p.part_id: p for p in parts}

    @functools.lru_cache()
    def _get_part_by_id(
        part_id: str,
        did: str,
        mid: str,
        eid: str,
        configuration: str = "",
    ) -> BTPartMetadataInfo:
        # TODO(RWS): Add filter to query parameters to reduce data consumption.
        parts = _get_parts(did, mid, eid, configuration)
        return parts[part_id]

    def _get_part_from_dict(part: Dict[str, Any]) -> BTPartMetadataInfo:
        # This is a convenience function because we can't hash on a dict.
        return _get_part_by_id(
            part_id=part["partId"],
            did=part["documentId"],
            mid=part["documentMicroversion"],
            eid=part["elementId"],
            configuration=part["configuration"],
        )

    def addPart(occurrence, matrix):
        """Add a part to the current robot link."""
        part = occurrence["instance"]
        part["name"] = renamePart(part["name"])
        part["configuration"] = renamePart(part["configuration"])

        if part["suppressed"]:
            return

        if part["partId"] == "":
            print(
                Fore.YELLOW
                + "WARNING: Part "
                + part["name"]
                + " has no partId"
                + Style.RESET_ALL
            )
            return

        # Importing STL file for this part
        justPart, prefix = extractPartName(part["name"], part["configuration"])

        extra = ""
        if occurrence["instance"]["configuration"] != "default":
            extra = (
                Style.DIM
                + " (configuration: "
                + occurrence["instance"]["configuration"]
                + ")"
            )
        symbol = "+"
        if partIsIgnore(justPart):
            symbol = "-"
            extra += Style.DIM + " / ignoring visual and collision"

        print(
            f"{Fore.GREEN}{symbol} Adding part "
            f"{occurrence['instance']['name']} with parent "
            f"{occurrenceNameById[occurrence['assignation']][0]}"
            f"{extra}{Style.RESET_ALL}"
        )

        material_tag = None
        if use_material_tags:
            new_part = _get_part_from_dict(part)
            material_tag = get_material_tag(new_part, config["materialTags"])
            if material_tag:
                material_tags_by_part_id[new_part.part_id] = material_tag
                print(
                    f"{Fore.LIGHTMAGENTA_EX}/ Found {material_tag} for part "
                    f"'{new_part.name}'{Style.RESET_ALL}"
                )
        if partIsIgnore(justPart):
            stlFile = None
        elif material_tags_only and not material_tag:
            stlFile = None
        else:
            stlFile = robot.meshDir / (prefix.replace("/", "_") + ".stl")
            # shorten the configuration to a maximum number of chars to prevent errors.
            # Necessary for standard parts like screws
            # TODO(RWS): Hashing seems fairly opaque.
            if len(part["configuration"]) > 40:
                shortend_configuration = hashlib.md5(
                    part["configuration"].encode("utf-8")
                ).hexdigest()
            else:
                shortend_configuration = part["configuration"]
            stl = client.part_studio_stl_m(
                part["documentId"],
                part["documentMicroversion"],
                part["elementId"],
                part["partId"],
                shortend_configuration,
            )
            with open(stlFile, "wb") as stream:
                stream.write(stl)

            stlMetadata = prefix.replace("/", "_") + ".part"
            with open(robot.meshDir / stlMetadata, "w", encoding="utf-8") as stream:
                json.dump(part, stream, indent=4, sort_keys=True)

        # Import the SCAD files pure shapes
        shapes = None
        if config["useScads"]:
            scadFile = prefix + ".scad"
            if os.path.exists(config["outputDirectory"] / scadFile):
                shapes = csg.process(
                    config["outputDirectory"] + "/" + scadFile,
                    config["pureShapeDilatation"],
                )

        # Obtain metadatas about part to retrieve color
        if config["color"] is not None:
            color = config["color"]
        else:
            metadata = client.part_get_metadata(
                part["documentId"],
                part["documentMicroversion"],
                part["elementId"],
                part["partId"],
                part["configuration"],
            )
            if "appearance" in metadata:
                colors = metadata["appearance"]["color"]
                alpha = metadata["appearance"]["opacity"]
                color = (
                    np.array([colors["red"], colors["green"], colors["blue"], alpha])
                    / 255.0
                )
                print(f"Part {part['name']} has color {color}.")
            else:
                color = [0.5, 0.5, 0.5, 1.0]

        # Obtain mass properties about that part
        if config["noDynamics"]:
            mass = 0
            com = [0] * 3
            inertia = [0] * 12
        else:
            if prefix in config["dynamicsOverride"]:
                entry = config["dynamicsOverride"][prefix]
                mass = entry["mass"]
                com = entry["com"]
                inertia = entry["inertia"]
            else:
                massProperties = client.part_mass_properties(
                    part["documentId"],
                    part["documentMicroversion"],
                    part["elementId"],
                    part["partId"],
                    part["configuration"],
                )

                if part["partId"] not in massProperties["bodies"]:
                    print(
                        Fore.YELLOW
                        + "WARNING: part "
                        + part["name"]
                        + " has no dynamics (maybe it is a surface)"
                        + Style.RESET_ALL
                    )
                    return
                massProperties = massProperties["bodies"][part["partId"]]
                try:
                    mass = massProperties["mass"][0]
                    com = massProperties["centroid"]
                    inertia = massProperties["inertia"]
                except (KeyError, IndexError):
                    # The part of type Surface has no mass properties.
                    mass = 1e-4
                    com = [0] * 3
                    inertia = [0] * 9

                if abs(mass) < 1e-4:
                    print(
                        Fore.YELLOW
                        + "WARNING: part "
                        + part["name"]
                        + " has no mass, maybe you should assign a material to it ?"
                        + Style.RESET_ALL
                    )

        pose = occurrence["transform"]
        if robot.relative:
            pose = np.linalg.inv(matrix) * pose

        robot.addPart(
            pose, stlFile, material_tag, mass, com, inertia, color, shapes, prefix
        )

    partNames = {}

    def extractPartName(name, configuration):
        parts = name.split(" ")
        del parts[-1]
        basePartName = "_".join(parts).lower()

        # only add configuration to name if its not default and not a very long
        # configuration (which happens for library parts like screws)
        if configuration != "default" and len(configuration) < 40:
            parts += ["_" + configuration.replace("=", "_").replace(" ", "_")]

        return basePartName, "_".join(parts).lower()

    def processPartName(name, configuration, overrideName=None):
        if overrideName is None:
            _, name = extractPartName(name, configuration)

            if name in partNames:
                partNames[name] += 1
            else:
                partNames[name] = 1

            if partNames[name] == 1:
                return name
            else:
                return name + "_" + str(partNames[name])
        else:
            return overrideName

    def buildRobot(tree, matrix):
        print(f"> Building robot for tree {occurrenceNameById[tree['id']][0]} ...")
        occurrence = getOccurrence([tree["id"]])
        instance = occurrence["instance"]
        print(
            Fore.BLUE
            + Style.BRIGHT
            + "* Adding top-level instance ["
            + instance["name"]
            + "]"
            + Style.RESET_ALL
        )

        # Find the link name, which could be on a child.

        # Build a part name that is unique but still informative
        link = processPartName(
            instance["name"], instance["configuration"], occurrence["linkName"]
        )

        # Create the link, collecting all children in the tree assigned to this
        # top-level part.
        robot.startLink(link, matrix)
        for occurrence in occurrences.values():
            if (
                occurrence["assignation"] == tree["id"]
                and occurrence["instance"]["type"] == "Part"
            ):
                addPart(occurrence, matrix)
            elif occurrence["assignation"] is not None:
                print(
                    f"Skipping occurrence {occurrence['instance']['name']} "
                    f"for tree {tree['id']} with assignation "
                    f"{occurrence['assignation']} and path {occurrence['path']}"
                )
        robot.endLink()

        # Adding the frames (linkage is relative to parent)
        if tree["id"] in frames:
            for name, part in frames[tree["id"]]:
                frame = getOccurrence(part)["transform"]
                if robot.relative:
                    frame = np.linalg.inv(matrix) * frame
                robot.addFrame(name, frame)
        print(
            f"Tree {occurrenceNameById[tree['id']][0]} has children: "
            f"{[occurrenceNameById[c['id']][0] for c in tree['children']]}"
        )

        # Following the children in the tree, calling this function recursively
        for child in tree["children"]:
            worldAxisFrame = child["axis_frame"]
            zAxis = child["z_axis"]
            jointType = child["jointType"]
            jointLimits = child["jointLimits"]

            if robot.relative:
                axisFrame = np.linalg.inv(matrix) * worldAxisFrame
                childMatrix = worldAxisFrame
            else:
                # In SDF format, everything is expressed in the world frame.
                # In this case, childMatrix will always be the identity matrix.
                axisFrame = worldAxisFrame
                childMatrix = matrix

            subLink = buildRobot(child, childMatrix)
            robot.addJoint(
                jointType,
                link,
                subLink,
                axisFrame,
                child["dof_name"],
                jointLimits,
                zAxis,
            )

        return link

    # Start building the robot
    print(f"tree: {tree}")
    buildRobot(tree, np.matrix(np.identity(4)))
    robot.finalize()
    # print(tree)

    print(
        f"\n{Style.BRIGHT}* Writing {robot.modelFormat.upper()} file{Style.RESET_ALL}"
    )
    output_directory = Path(config["outputDirectory"])
    output_directory.mkdir(parents=True, exist_ok=True)
    if robot.createRosPackage:
        ros_package.generate_package(
            robot.packageName,
            robot.packageType,
            robot.robotName,
            robot.modelFormat,
            output_directory,
        )
    robot.write(robot.modelFilePath)

    if len(config["postImportCommands"]):
        print(f"\n{Style.BRIGHT}* Executing post-import commands{Style.RESET_ALL}")
        for command in config["postImportCommands"]:
            print(f"* {command}")
            os.system(command)


if __name__ == "__main__":
    main()
