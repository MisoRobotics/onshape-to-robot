import dataclasses
import math
import uuid
from collections import defaultdict, deque
from sys import exit
from typing import Final

import debugpy
import numpy as np
from colorama import Back, Fore, Style
from onshape_client import OnshapeElement

from .config import config, configFile
from .onshape_api.client import Client, get_assembly_from_url

# debugpy.listen(5678)
# debugpy.wait_for_client()

# OnShape API client
workspaceId = None
client = Client(logging=False, creds=configFile)
client.useCollisionsConfigurations = config["useCollisionsConfigurations"]

if config["documentsUrl"]:
    print(
        f"\n{Style.BRIGHT}* Using Documents API URL {config['documentsUrl']}...{Style.RESET_ALL}"
    )
    root_assembly: Final[OnshapeElement] = OnshapeElement(config["documentsUrl"])

    source = get_assembly_from_url(config["documentsUrl"])
    assembly = client.get_assembly(
        source["did"],
        source["wid"],
        source["eid"],
        source["type"],
        configuration=config["configuration"],
    )
    config["documentId"] = source["did"]
    if source["type"] == "v":
        config["versionId"] = source["wid"]
    else:
        config["workspaceId"] = source["wid"]
else:
    assembly = None
    raise Exception("Only documentsUrl is supported.")

# If a versionId is provided, it will be used, else the main workspace is retrieved
if config["versionId"] != "":
    print(
        "\n"
        + Style.BRIGHT
        + "* Using configuration version ID "
        + config["versionId"]
        + " ..."
        + Style.RESET_ALL
    )
elif config["workspaceId"] != "":
    print(
        "\n"
        + Style.BRIGHT
        + "* Using configuration workspace ID "
        + config["workspaceId"]
        + " ..."
        + Style.RESET_ALL
    )
    workspaceId = config["workspaceId"]
else:
    print("\n" + Style.BRIGHT + "* Retrieving workspace ID ..." + Style.RESET_ALL)
    response = client.get_document(config["documentId"]).json()
    workspaceId = response["defaultWorkspace"]["id"]
    config["workspaceId"] = workspaceId
    print(Fore.GREEN + "+ Using workspace id: " + workspaceId + Style.RESET_ALL)

# Now, finding the assembly, according to given name in configuration, or else the first possible one
print(
    "\n"
    + Style.BRIGHT
    + "* Retrieving elements in the document, searching for the assembly..."
    + Style.RESET_ALL
)
if config["versionId"] != "":
    elements = client.list_elements(
        config["documentId"], config["versionId"], "v"
    ).json()
else:
    elements = client.list_elements(config["documentId"], workspaceId).json()
assemblyId = None
assemblyName = ""
for element in elements:
    if element["type"] == "Assembly" and (
        config["assemblyName"] is False or element["name"] == config["assemblyName"]
    ):
        print(
            Fore.GREEN
            + "+ Found assembly, id: "
            + element["id"]
            + ', name: "'
            + element["name"]
            + '"'
            + Style.RESET_ALL
        )
        assemblyName = element["name"]
        assemblyId = element["id"]

if assemblyId == None:
    print(
        Fore.RED + "ERROR: Unable to find assembly in this document" + Style.RESET_ALL
    )
    exit(1)

# Retrieving the assembly
print(f"\n{Style.BRIGHT}* Retrieving assembly {root_assembly.name}...{Style.RESET_ALL}")
assembly = assembly or client.get_assembly(
    root_assembly.did,
    root_assembly.wvmid,
    root_assembly.eid,
    root_assembly.wvm,
    configuration=config["configuration"],
)

root = assembly["rootAssembly"]


def order_occurrences(occurrence_a, occurrence_b):
    """Returns the occurrence in the order of child to parent."""
    child = None
    parent = None

    if occurrence_a in transformed_edges_mapping[occurrence_b]:
        child = occurrence_a
        parent = occurrence_b
    else:
        if occurrence_b not in transformed_edges_mapping[occurrence_a]:
            raise RuntimeError("Rerooting of tree performed incorrectly.")
        child = occurrence_b
        parent = occurrence_a
    return child, parent


# Finds a (leaf) instance given the full path, typically A B C where A and B would be subassemblies and C
# the final part


def findInstance(path, instances=None):
    global assembly

    if instances is None:
        instances = assembly["rootAssembly"]["instances"]

    for instance in instances:
        if instance["id"] == path[0]:
            if len(path) == 1:
                # If the length of remaining path is 1, the part is in the current assembly/subassembly
                return instance
            else:
                # Else, we need to find the matching sub assembly to find the proper part (recursively)
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

# Gets an occurrence given its full path


def getOccurrence(path):
    return occurrences[tuple(path)]


# Assignations are pieces that will be in the same link. Note that this is only for top-level
# item of the path (all sub assemblies and parts in assemblies are naturally in the same link as
# the parent), but other parts that can be connected with mates in top assemblies are then assigned to
# the link
assignations = {}

# Frames (mated with frame_ name) will be special links in the output file allowing to track some specific
# manually identified frames
frames = defaultdict(list)


def assignParts(child, parent=None):
    if parent is None:
        # TODO(RWS): Don't leak 'frame'; it's currently the only use case.
        parent = "frame"
    elif child != parent:
        child, parent = order_occurrences(child, parent)

    assignations[child] = parent
    for occurrence in occurrences.values():
        if occurrence["path"][0] == child:
            occurrence["assignation"] = parent
    return child, parent


def connectParts(child, parent):
    assignParts(child, parent)


from .features import FeatureSource
from .features import init as features_init

features_init(client, config, root, workspaceId, assemblyId)

FEATURES: FeatureSource = FeatureSource(client._api.onshape_client, root_assembly)

# Build tree.
edges = set()
features = root["features"]
for feature in features:
    if feature["featureType"] == "mateConnector":
        continue
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

    occurrence_a = data["matedEntities"][0]["matedOccurrence"][0]
    occurrence_b = data["matedEntities"][1]["matedOccurrence"][0]
    edges.add((occurrence_a, occurrence_b))

# Helper function to reroot tree.
def reroot_tree(edges, root):
    """Update direction of edges given a specified root using BFS."""
    # print(f"Initial graph: {edges}")
    new_edges = set()
    nodes = set()
    node_neighbours = defaultdict(list)
    # Mapping for neighbours (ignoring directionality).
    for i, j in edges:
        nodes.add(i)
        nodes.add(j)
        node_neighbours[i].append(j)
        node_neighbours[j].append(i)

    assert root in nodes, "Root not in provided graph."
    # BFS, along with switching the direction of applicable edges.
    q = deque([root])
    seen = set()
    while q:
        curr_node = q.popleft()
        if curr_node in seen:
            continue
        seen.add(curr_node)

        for curr_neighbour in node_neighbours[curr_node]:
            if curr_neighbour not in seen:
                # edges is a set with either (curr_neighbour, node) or (node, curr_neighbour)
                new_edges.add((curr_neighbour, curr_node))
                q.append(curr_neighbour)
    # print(f"Final graph: {new_edges}")
    return new_edges, nodes


# Looking for trunk node.
trunk = None
for feature in features:
    if feature["featureType"] == "mateConnector":
        name = feature["featureData"]["name"]
        if name == "trunk":
            trunk = feature["featureData"]["occurrence"][0]
            print(f"Found a trunk on part {occurrenceNameById[trunk]}")
            break
if trunk is not None:
    transformed_edges, all_nodes = reroot_tree(edges, trunk)


# Mapping where the key is the parent and the value is a list of its children.
transformed_edges_mapping = defaultdict(list)
for edge in transformed_edges:
    transformed_edges_mapping[edge[1]].append(edge[0])
# print("transformed_edges", transformed_edges)

# First, features are scanned to find the DOFs. Links that they connects are then tagged
print(
    "\n"
    + Style.BRIGHT
    + "* Getting assembly features, scanning for DOFs..."
    + Style.RESET_ALL
)
relations = {}
features = root["features"]
for feature in features:
    if feature["featureType"] == "mateConnector":
        name = feature["featureData"]["name"]
        if name[0:5] == "link_":
            name = name[5:]
            occurrences[(feature["featureData"]["occurrence"][0],)]["linkName"] = name
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

        occurrence_a = data["matedEntities"][0]["matedOccurrence"][0]
        occurrence_b = data["matedEntities"][1]["matedOccurrence"][0]

        child, parent = order_occurrences(occurrence_a, occurrence_b)

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
                    "       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported"
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

            assignParts(child, child)
            assignParts(parent, parent)

            if child in relations:
                print(Fore.RED)
                print(
                    "Error, the relation "
                    + name
                    + " is connected a child that is already connected"
                )
                print("Be sure you ordered properly your relations, see:")
                print(
                    "https://onshape-to-robot.readthedocs.io/en/latest/design.html#specifying-degrees-of-freedom"
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

print(
    Fore.GREEN
    + Style.BRIGHT
    + "* Found total "
    + str(len(relations))
    + " DOFs"
    + Style.RESET_ALL
)

# If we have no DOF
if len(relations) == 0:
    if trunk is None:
        trunk = root["instances"][0]["id"]
    assignParts(trunk, trunk)


def appendFrame(key, frame):
    child_name = occurrenceNameById[frame[1][-1]]
    parent_name = occurrenceNameById[key]
    print(f"- Appending {child_name} as frame {frame[0]} to {parent_name}")
    frames[key].append(frame)


# Spreading parts assignations, this parts mainly does two things:
# 1. Finds the parts of the top level assembly that are not directly in a sub assembly and try to assign them
#    to an existing link that was identified before
# 2. Among those parts, finds the ones that are frames (connected with a frame_* connector)

for feature in features:
    if feature["featureType"] != "mate" or feature["suppressed"]:
        continue

    data = feature["featureData"]

    if (
        len(data["matedEntities"]) != 2
        or len(data["matedEntities"][0]["matedOccurrence"]) == 0
        or len(data["matedEntities"][1]["matedOccurrence"]) == 0
    ):
        continue

    occurrence_a = data["matedEntities"][0]["matedOccurrence"][0]
    occurrence_b = data["matedEntities"][1]["matedOccurrence"][0]
    child, parent = order_occurrences(occurrence_a, occurrence_b)
    assert (child, parent) == order_occurrences(occurrence_b, occurrence_a)

    if data["name"].startswith("frame_"):
        child_path = occurrenceById[child]["path"]
        # For frames, connect the parentmost entity instead of the child part.
        child = child_path[0]
        name = data["name"][len("frame_") :]
        appendFrame(parent, [name, child_path])
        # TODO: Revisit - unsure of the functionality here.
        parent = assignations[parent] if config["drawFrames"] else None
    connectParts(child, parent)

# Building and checking robot tree, here we:
# 1. Search for robot trunk (which will be the top-level link)
# 2. Scan for orphaned parts (if you add something floating with no mate to anything)
#    that are then assigned to trunk by default
# 3. Collect all the pieces of the robot tree
print("\n" + Style.BRIGHT + "* Building robot tree" + Style.RESET_ALL)

if trunk is None:
    for childId in relations:
        entry = relations[childId]
        if entry["parent"] not in relations:
            trunk = entry["parent"]
            break
trunkOccurrence = getOccurrence([trunk])
print(
    Style.BRIGHT + "* Trunk is " + trunkOccurrence["instance"]["name"] + Style.RESET_ALL
)

for occurrence in occurrences.values():
    if occurrence["assignation"] is None:
        print(
            Fore.YELLOW
            + "WARNING: part ("
            + occurrence["instance"]["name"]
            + ") has no assignation, connecting it with trunk"
            + Style.RESET_ALL
        )
        child = occurrence["path"][0]
        # debugpy.breakpoint()
        # import pudb; pudb.set_trace()
        connectParts(child, trunk)


def collect(id):
    part = {}
    part["id"] = id
    part["child?n"] = []
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
