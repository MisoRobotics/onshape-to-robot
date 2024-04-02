from collections import defaultdict
from typing import Final

import numpy as np
from colorama import (
    Fore,
    Style,
)
from onshape_client import OnshapeElement
from onshape_to_robot.config import load_config
from onshape_to_robot.features import FeatureSource
from onshape_to_robot.features import init as features_init

from .onshape_api.client import Client

config = load_config()
configFile = config.configPath

# OnShape API client
workspaceId = None
client = Client(logging=False, creds=configFile)
client.use_collisions_configurations = config["useCollisionsConfigurations"]

if not config["documentsUrl"]:
    raise Exception("Only documentsUrl is supported.")

print(
    f"\n{Style.BRIGHT}* Using Onshape Documents API URL "
    f"{config['documentsUrl']}{Style.RESET_ALL}"
)
element: Final[OnshapeElement] = OnshapeElement(config["documentsUrl"])
assembly = client.get_assembly(
    element.did,
    element.wvmid,
    element.eid,
    element.wvm,
    configuration=config["configuration"],
)
config["documentId"] = element.did
if element.wvm == "v":
    config["versionId"] = element.wvmid
else:
    config["workspaceId"] = element.wvmid

root = assembly["rootAssembly"]
assemblyName = element.name
assemblyId = element.eid

print(f"\n{Style.BRIGHT}* Retrieved assembly '{assemblyName}'.{Style.RESET_ALL}")


def findInstance(path, instances=None):
    # Finds a (leaf) instance given the full path, typically A B C where A and B would
    # be subassemblies and C the final part

    global assembly

    if instances is None:
        instances = assembly["rootAssembly"]["instances"]

    for instance in instances:
        if instance["id"] == path[0]:
            if len(path) == 1:
                # If the length of remaining path is 1, the part is in the current
                # assembly/subassembly
                return instance
            else:
                # Else, we need to find the matching sub assembly to find the proper
                # part (recursively)
                d = instance["documentId"]
                m = instance["documentMicroversion"]
                e = instance["elementId"]
                for asm in assembly["subAssemblies"]:
                    if (
                        asm["documentId"] == d
                        and asm["documentMicroversion"] == m
                        and asm["elementId"] == e
                    ):
                        return findInstance(path[1:], asm["instances"])

    print(Fore.RED + "Could not find instance for " + str(path) + Style.RESET_ALL)


# Collecting occurrences, the path is the assembly / sub assembly chain
occurrences = {}
for occurrence in root["occurrences"]:
    occurrence["assignation"] = None
    occurrence["instance"] = findInstance(occurrence["path"])
    occurrence["transform"] = np.matrix(np.reshape(occurrence["transform"], (4, 4)))
    occurrence["linkName"] = None
    occurrences[tuple(occurrence["path"])] = occurrence

occurrenceById = {o["instance"]["id"]: o for o in occurrences.values()}
occurrenceNameById = {
    o["instance"]["id"]: [occurrenceById[f]["instance"]["name"] for f in o["path"]]
    for o in occurrences.values()
}


features_init(client, config, root, workspaceId, assemblyId)

FEATURES: FeatureSource = FeatureSource(client._api.onshape_client, element)


def getOccurrence(path):
    """Get an occurrence from its full path."""
    if isinstance(path, str):
        path = tuple((path,))
    try:
        return occurrences[tuple(path)]
    except KeyError:
        return occurrenceById[path[-1]]


def occurrence_is_suppressed(path):
    """Return true if an occurrence is suppressed.

    An occurrence should be considered suppressed if is marked as
    suppressed OR if its parent or its parent's parent (and so on up the
    chain to the root assembly) is marked suppressed.

    Args:
        path (tuple(str)): The path of the occurrence from its own id
            up to the root assembly.

    Returns:
        bool: True if the occurrence is suppressed; false otherwise.
    """
    this_occurrence = getOccurrence(path)
    try:
        for this_occurrence_id in this_occurrence["path"]:
            if occurrenceById[this_occurrence_id]["instance"]["suppressed"]:
                return True
    except KeyError as exc:
        print(f"path: {path}")
        raise exc
    return False


# Assignations are pieces that will be in the same link. Note that this is only for
# top-level item of the path (all sub assemblies and parts in assemblies are naturally
# in the same link as the parent), but other parts that can be connected with mates in
# top assemblies are then assigned to the link.
assignations = {}

# Frames (mated with frame_ name) will be special links in the output file allowing to
# track some specific manually identified frames
frames = defaultdict(list)


def assignParts(root, parent):
    assignations[root] = parent
    root_occ = occurrenceById[root]
    if root_occ["assignation"] == parent:
        return False
    else:
        root_occ["assignation"] = parent
        return True


def connectParts(child, parent):
    return assignParts(child, parent)


# Scan mates and mate connectors for specific names.
print(f"\n{Style.BRIGHT}Parsing tags from assembly...{Style.RESET_ALL}")
trunk = None
tagged_trunk = None
relations = {}
links = {}
features = root["features"]
for feature in features:
    if feature["featureType"] == "mateConnector":
        name = feature["featureData"]["name"]
        print(feature["featureData"])
        try:
            path = (feature["featureData"]["occurrence"][0],)
        except IndexError:
            # NOTE: Mate connectors on the origin have no occurrence.
            print(f"{Fore.YELLOW}Skipping mate connector {name}{Style.RESET_ALL}")
            continue
        if name.startswith("link_"):
            name = name[len("link_") :]
            occurrences[path]["linkName"] = name
            links[name] = path
            print(f"{Fore.GREEN}+ Found link: {name}{Style.RESET_ALL}")
        elif name == "trunk":
            print(f"{Fore.GREEN}+ Found tagged trunk: {name}{Style.RESET_ALL}")
            tagged_trunk = path[-1]
    else:
        if feature["suppressed"]:
            continue

        data = feature["featureData"]

        if (
            "matedEntities" not in data
            or len(data["matedEntities"]) != 2
            or len(data["matedEntities"][0]["matedOccurrence"]) == 0
            or len(data["matedEntities"][1]["matedOccurrence"]) == 0
        ):
            continue

        child = data["matedEntities"][0]["matedOccurrence"][0]
        parent = data["matedEntities"][1]["matedOccurrence"][0]

        if data["name"][0:3] == "dof":
            parts = data["name"].split("_")
            del parts[0]
            data["inverted"] = False
            if parts[-1] == "inv" or parts[-1] == "inverted":
                data["inverted"] = True
                del parts[-1]
            name = "_".join(parts)
            if name == "":
                print(
                    Fore.RED
                    + "ERROR: a DOF dones't have any name (\""
                    + data["name"]
                    + '" should be "dof_...")'
                    + Style.RESET_ALL
                )
                exit()

            limits = None
            if data["mateType"] == "REVOLUTE" or data["mateType"] == "CYLINDRICAL":
                jointType = "revolute"
                if not config["ignoreLimits"]:
                    limits = FEATURES.get_limits(data["name"])
            elif data["mateType"] == "SLIDER":
                jointType = "prismatic"
                if not config["ignoreLimits"]:
                    limits = FEATURES.get_limits(data["name"])
            elif data["mateType"] == "FASTENED":
                jointType = "fixed"
            else:
                print(
                    Fore.RED
                    + 'ERROR: "'
                    + name
                    + '" is declared as a DOF but the mate type is '
                    + data["mateType"]
                    + ""
                )
                print(
                    "   Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported"
                    + Style.RESET_ALL
                )
                exit(1)

            # We compute the axis in the world frame
            matedEntity = data["matedEntities"][0]
            matedTransform = getOccurrence(matedEntity["matedOccurrence"])["transform"]

            # jointToPart is the (rotation only) matrix from joint to the part
            # it is attached to
            jointToPart = np.eye(4)
            jointToPart[:3, :3] = np.stack(
                (
                    np.array(matedEntity["matedCS"]["xAxis"]),
                    np.array(matedEntity["matedCS"]["yAxis"]),
                    np.array(matedEntity["matedCS"]["zAxis"]),
                )
            ).T

            if data["inverted"]:
                if limits is not None:
                    limits = (-limits[1], -limits[0])

                # Flipping the joint around X axis
                flip = np.array(
                    [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                )
                jointToPart = jointToPart.dot(flip)

            zAxis = np.array([0, 0, 1])

            origin = matedEntity["matedCS"]["origin"]
            translation = np.matrix(np.identity(4))
            translation[0, 3] += origin[0]
            translation[1, 3] += origin[1]
            translation[2, 3] += origin[2]
            worldAxisFrame = matedTransform * translation

            # Resulting frame of axis, always revolving around z
            worldAxisFrame = worldAxisFrame.dot(jointToPart)

            limitsStr = ""
            if limits is not None:
                limitsStr = (
                    "["
                    + str(round(limits[0], 3))
                    + ": "
                    + str(round(limits[1], 3))
                    + "]"
                )
            print(
                Fore.GREEN
                + "+ Found DOF: "
                + name
                + " "
                + Style.DIM
                + "("
                + jointType
                + ")"
                + limitsStr
                + Style.RESET_ALL
            )

            if child in relations:
                print(Fore.RED)
                print(
                    "Error, the relation "
                    + name
                    + " is connected a child that is already connected"
                )
                print("Be sure you ordered properly your relations, see:")
                print(
                    "https://onshape-to-robot.readthedocs.io/en/latest/design.html"
                    "#specifying-degrees-of-freedom"
                )
                print(Style.RESET_ALL)
                exit()

            relations[child] = {
                "parent": parent,
                "worldAxisFrame": worldAxisFrame,
                "zAxis": zAxis,
                "name": name,
                "type": jointType,
                "limits": limits,
            }

            assignParts(child, child)
            assignParts(parent, parent)

if tagged_trunk:
    trunk_name = getOccurrence([tagged_trunk])["linkName"]
    print(f"{Fore.GREEN}+ Found trunk on link: {trunk_name}{Style.RESET_ALL}")

print(
    f"{Fore.GREEN}{Style.BRIGHT}* Found {len(relations)} named dofs(s) and "
    f"{len(links)} named link(s).{Style.RESET_ALL}"
)

# The tagged trunk should take precedence.
trunk = trunk or tagged_trunk

# If we have no DOF
if len(relations) == 0:
    trunk = trunk or root["instances"][0]["id"]
if trunk:
    assignParts(trunk, trunk)


def appendFrame(key, frame):
    child_name = occurrenceNameById[frame[1][-1]][0]
    parent_name = occurrenceNameById[key][0]
    print(f"- Appending {child_name} as frame {frame[0]} to {parent_name}")
    frames[key].append(frame)


# Spreading parts assignations, this parts mainly does two things:
# 1. Finds the parts of the top level assembly that are not directly in a sub assembly
#    and try to assign them to an existing link that was identified before
# 2. Among those parts, finds the ones that are frames (connected with a frame_*
#     connector)
changed = True
while changed:
    changed = False
    for feature in features:
        if feature["suppressed"]:
            continue

        data = feature["featureData"]
        if feature["featureType"] == "mateGroup":
            parent = None
            for occurrence in data["occurrences"]:
                if occurrence["occurrence"][0] in assignations:
                    parent = occurrence["occurrence"][0]
                    break
            if parent is None:
                continue
            connectParts(parent, parent)
            for occurrence in occurrences.values():
                print(occurrence)
                try:
                    if occurrence_is_suppressed(occurrence["path"]):
                        continue
                except KeyError:
                    print(f"occurrence: {occurrence}")
                    raise
                if occurrence["path"][0] != parent:
                    continue
                child = occurrence["instance"]["id"]
                if child != parent:
                    changed |= connectParts(child, parent)
                    if changed:
                        child_name = occurrenceById[child]["instance"]["name"]
                        parent_name = occurrenceById[parent]["instance"]["name"]
                        print(
                            f"Connected {child_name} ({child}) to {parent_name} ({parent})"
                        )

        if feature["featureType"] != "mate":
            continue

        if (
            len(data["matedEntities"]) != 2
            or len(data["matedEntities"][0]["matedOccurrence"]) == 0
            or len(data["matedEntities"][1]["matedOccurrence"]) == 0
        ):
            try:
                if data["name"].startswith("frame_"):
                    raise Exception(
                        f"Found mate relation {data['name']}, but it did not "
                        "have two mated entities (had "
                        f"{len(data['matedEntities'])} instead). This can "
                        "happen if you use Mate Connectors defined in a Part "
                        "Studio instead of in an assembly. Try adding Mate "
                        "Connectors to a subassembly or the root assembly "
                        "and use them in the mate relation."
                    )
            except IndexError:
                pass
            continue

        is_frame = data["name"].startswith("frame_")
        if is_frame:
            name = "_".join(data["name"].split("_")[1:])

        occurrenceA = data["matedEntities"][0]["matedOccurrence"][0]
        occurrenceB = data["matedEntities"][1]["matedOccurrence"][0]

        if occurrenceA in assignations and occurrenceB not in assignations:
            parent_index, child_index = 0, 1
        elif occurrenceA not in assignations and occurrenceB in assignations:
            parent_index, child_index = 1, 0
        else:
            continue

        changed = True

        # Get the parent occurrence, which may have been reassigned.
        parent_id = occurrenceById[
            data["matedEntities"][parent_index]["matedOccurrence"][-1]
        ]["assignation"]
        child_path = data["matedEntities"][child_index]["matedOccurrence"]
        child_root_id = child_path[0]

        if is_frame:
            parent_id = parent_id or trunk
            appendFrame(parent_id, [name, child_path])
            parent = assignations[parent_id] if config["drawFrames"] else "frame"
            assignParts(child_root_id, parent)
        else:
            if occurrenceA in assignations:
                connectParts(occurrenceB, assignations[occurrenceA])

            else:
                connectParts(occurrenceA, assignations[occurrenceB])


# Building and checking robot tree, here we:
# 1. Search for robot trunk (which will be the top-level link)
# 2. Scan for orphaned parts (if you add something floating with no mate to anything)
#    that are then assigned to trunk by default
# 3. Collect all the pieces of the robot tree
print("\n" + Style.BRIGHT + "* Building robot tree" + Style.RESET_ALL)

possible_trunks = []
for childId in relations:
    entry = relations[childId]
    if entry["parent"] not in relations:
        possible_trunks += [entry["parent"]]

print(f"Found {len(possible_trunks)} possible trunks.")
trunk = trunk or possible_trunks[0]
print(f"Selected trunk: '{occurrenceNameById[trunk][0]}'")

# Go through each occurrence and bubble up link name.
for child, root in assignations.items():
    try:
        root_occurrence = getOccurrence(root)
    except KeyError:
        continue
    child_name = getOccurrence(child)["linkName"]
    root_name = root_occurrence["linkName"]
    if child_name:
        if root_name and root_name != child_name:
            raise ValueError(
                f"Found link tag '{child_name}' on child, but root "
                f"already had tag '{root_name}'."
            )
        root_occurrence["linkName"] = child_name
        print(f"Renamed {occurrenceNameById[root][0]} to '{child_name}'.")

trunkOccurrence = getOccurrence([trunk])
print(
    Style.BRIGHT + "* Trunk is " + trunkOccurrence["instance"]["name"] + Style.RESET_ALL
)

for occurrence in occurrences.values():
    if occurrence["assignation"] is not None:
        continue
    if occurrence_is_suppressed(occurrence["path"]):
        continue

    name = occurrence["instance"]["name"]
    parent = occurrence["path"][0]
    if parent not in assignations:
        parent = trunk

    parent_name = occurrenceNameById[parent]
    print(
        f"{Fore.YELLOW}WARNING: part ({name}) has no assignation, "
        f"connecting it with {parent_name}{Style.RESET_ALL}"
    )
    child = occurrence["instance"]["id"]
    connectParts(child, parent)


def collect(id):
    part = {}
    part["id"] = id
    part["children"] = []
    for childId in relations:
        entry = relations[childId]
        if entry["parent"] == id:
            child = collect(childId)
            child["axis_frame"] = entry["worldAxisFrame"]
            child["z_axis"] = entry["zAxis"]
            child["dof_name"] = entry["name"]
            child["jointType"] = entry["type"]
            child["jointLimits"] = entry["limits"]
            part["children"].append(child)
    return part


tree = collect(trunk)
