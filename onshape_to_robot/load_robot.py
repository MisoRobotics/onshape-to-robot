import math
import uuid
from collections import defaultdict
from sys import exit

import numpy as np
from colorama import Back, Fore, Style

from .config import config, configFile
from .onshape_api.client import Client, get_assembly_from_url

# OnShape API client
workspaceId = None
client = Client(logging=False, creds=configFile)
client.useCollisionsConfigurations = config['useCollisionsConfigurations']

if config['documentUrl']:
    print("\n" + Style.BRIGHT + '* Using Documents API URL ' +
          config['documentUrl']+' ...' + Style.RESET_ALL)
    assemblySource = get_assembly_from_url(config['documentUrl'])
else:
    # If a versionId is provided, it will be used, else the main workspace is retrieved
    if config['versionId'] != '':
        print("\n" + Style.BRIGHT + '* Using configuration version ID ' +
            config['versionId']+' ...' + Style.RESET_ALL)
    elif config['workspaceId'] != '':
        print("\n" + Style.BRIGHT + '* Using configuration workspace ID ' +
            config['workspaceId']+' ...' + Style.RESET_ALL)
        workspaceId = config['workspaceId']
    else:
        print("\n" + Style.BRIGHT + '* Retrieving workspace ID ...' + Style.RESET_ALL)
        document = client.get_document(config['documentId']).json()
        workspaceId = document['defaultWorkspace']['id']
        print(Fore.GREEN + "+ Using workspace id: " + workspaceId + Style.RESET_ALL)

    # Now, finding the assembly, according to given name in configuration, or else the first possible one
    print("\n" + Style.BRIGHT +
        '* Retrieving elements in the document, searching for the assembly...' + Style.RESET_ALL)
    if config['versionId'] != '':
        elements = client.list_elements(
            config['documentId'], config['versionId'], 'v').json()
    else:
        elements = client.list_elements(config['documentId'], workspaceId).json()
    assemblyId = None
    assemblyName = ''
    for element in elements:
        if element['type'] == 'Assembly' and \
                (config['assemblyName'] is False or element['name'] == config['assemblyName']):
            print(Fore.GREEN + "+ Found assembly, id: " +
                element['id']+', name: "'+element['name']+'"' + Style.RESET_ALL)
            assemblyName = element['name']
            assemblyId = element['id']

    if assemblyId == None:
        print(Fore.RED + "ERROR: Unable to find assembly in this document" + Style.RESET_ALL)
        exit(1)
    assemblySource = {
        "did": config['documentId'],
        "wid": config['versionId'] or workspaceId,
        "eid": assemblyId,
        "type": "v" if config['versionId'] else "w",
    }

assemblyId = assemblySource['eid']
workspaceId = assemblySource['wid']

# Retrieving the assembly
print("\n" + Style.BRIGHT + '* Retrieving assembly '+ assemblyId + Style.RESET_ALL)
assemblySource["configuration"] = config['configuration']
assembly = client.get_assembly(**assemblySource)
root = assembly['rootAssembly']

# Finds a (leaf) instance given the full path, typically A B C where A and B would be subassemblies and C
# the final part


def findInstance(path, instances=None):
    global assembly

    if instances is None:
        instances = assembly['rootAssembly']['instances']

    for instance in instances:
        if instance['id'] == path[0]:
            if len(path) == 1:
                # If the length of remaining path is 1, the part is in the current assembly/subassembly
                return instance
            else:
                # Else, we need to find the matching sub assembly to find the proper part (recursively)
                d = instance['documentId']
                m = instance['documentMicroversion']
                e = instance['elementId']
                for asm in assembly['subAssemblies']:
                    if asm['documentId'] == d and asm['documentMicroversion'] == m and asm['elementId'] == e:
                        return findInstance(path[1:], asm['instances'])

    print(Fore.RED + 'Could not find instance for ' + str(path) + Style.RESET_ALL)


# Collecting occurrences, the path is the assembly / sub assembly chain
occurrences = {}
for occurrence in root['occurrences']:
    occurrence['assignation'] = None
    occurrence['instance'] = findInstance(occurrence['path'])
    occurrence['transform'] = np.matrix(
        np.reshape(occurrence['transform'], (4, 4)))
    occurrence['linkName'] = None
    occurrences[tuple(occurrence['path'])] = occurrence

occurrencesByName = {
    o['instance']['name']: o for o in occurrences.values()
}
occurrencesById = {
    o['instance']['id']: o['instance']['name'] for o in occurrences.values()
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


def assignParts(root, parent):
    assignations[root] = parent
    for occurrence in occurrences.values():
        if occurrence['path'][0] == root:
            occurrence['assignation'] = parent


def connectParts(child, parent):
    print(f"> Connecting child {child} to parent {parent}")
    assignParts(child, parent)


from .features import getLimits
from .features import init as features_init

features_init(client, config, root, workspaceId, assemblyId)

# First, features are scanned to find the DOFs. Links that they connects are then tagged
print("\n" + Style.BRIGHT +
      '* Getting assembly features, scanning for DOFs...' + Style.RESET_ALL)
trunk = None
relations = {}
features = root['features']
for feature in features:
    if feature['featureType'] == 'mateConnector':
        name = feature['featureData']['name']
        if name[0:5] == 'link_':
            name = name[5:]
            occurrences[(feature['featureData']['occurrence'][0],)
                        ]['linkName'] = name
    else:
        if feature['suppressed']:
            continue

        data = feature['featureData']

        if 'matedEntities' not in data or len(data['matedEntities']) != 2 or \
                len(data['matedEntities'][0]['matedOccurrence']) == 0 \
                or len(data['matedEntities'][1]['matedOccurrence']) == 0:
            continue

        child = data['matedEntities'][0]['matedOccurrence'][0]
        parent = data['matedEntities'][1]['matedOccurrence'][0]

        if data['name'][0:3] == 'dof':
            parts = data['name'].split('_')
            del parts[0]
            data['inverted'] = False
            if parts[-1] == 'inv' or parts[-1] == 'inverted':
                data['inverted'] = True
                del parts[-1]
            name = '_'.join(parts)
            if name == '':
                print(Fore.RED +
                      'ERROR: a DOF dones\'t have any name ("'+data['name']+'" should be "dof_...")' + Style.RESET_ALL)
                exit()

            limits = None
            if data['mateType'] == 'REVOLUTE' or data['mateType'] == 'CYLINDRICAL':
                jointType = 'revolute'

                if not config['ignoreLimits']:
                    limits = getLimits(jointType, data['name'])
            elif data['mateType'] == 'SLIDER':
                jointType = 'prismatic'
                if not config['ignoreLimits']:
                    limits = getLimits(jointType, data['name'])
            elif data['mateType'] == 'FASTENED':
                jointType = 'fixed'
            else:
                print(Fore.RED + 'ERROR: "' + name +
                      '" is declared as a DOF but the mate type is '+data['mateType']+'')
                print(
                    '       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported' + Style.RESET_ALL)
                exit(1)

            # We compute the axis in the world frame
            matedEntity = data['matedEntities'][0]
            matedTransform = getOccurrence(
                matedEntity['matedOccurrence'])['transform']

            # jointToPart is the (rotation only) matrix from joint to the part
            # it is attached to
            jointToPart = np.eye(4)
            jointToPart[:3, :3] = np.stack((
                np.array(matedEntity['matedCS']['xAxis']),
                np.array(matedEntity['matedCS']['yAxis']),
                np.array(matedEntity['matedCS']['zAxis'])
            )).T

            if data['inverted']:
                if limits is not None:
                    limits = (-limits[1], -limits[0])


                # Flipping the joint around X axis
                flip = np.array([[1, 0, 0, 0],
                                [0, -1, 0, 0],
                                [0, 0, -1, 0],
                                [0, 0,  0,  1]])
                jointToPart = jointToPart.dot(flip)

            zAxis = np.array([0, 0, 1])

            origin = matedEntity['matedCS']['origin']
            translation = np.matrix(np.identity(4))
            translation[0, 3] += origin[0]
            translation[1, 3] += origin[1]
            translation[2, 3] += origin[2]
            worldAxisFrame = matedTransform * translation

            # Resulting frame of axis, always revolving around z
            worldAxisFrame = worldAxisFrame.dot(jointToPart)

            limitsStr = ''
            if limits is not None:
                limitsStr = '[' + str(round(limits[0], 3)) + \
                    ': ' + str(round(limits[1], 3)) + ']'
            print(Fore.GREEN + '+ Found DOF: '+name + ' ' + Style.DIM +
                  '('+jointType+')'+limitsStr + Style.RESET_ALL)

            if child in relations:
                print(Fore.RED)
                print('Error, the relation '+name +
                      ' is connected a child that is already connected')
                print('Be sure you ordered properly your relations, see:')
                print(
                    'https://onshape-to-robot.readthedocs.io/en/latest/design.html#specifying-degrees-of-freedom')
                print(Style.RESET_ALL)
                exit()

            relations[child] = {
                'parent': parent,
                'worldAxisFrame': worldAxisFrame,
                'zAxis': zAxis,
                'name': name,
                'type': jointType,
                'limits': limits
            }

            assignParts(child, child)
            assignParts(parent, parent)

print(Fore.GREEN + Style.BRIGHT + '* Found total ' +
      str(len(relations))+' DOFs' + Style.RESET_ALL)

# If we have no DOF
if len(relations) == 0:
    trunk = root['instances'][0]['id']
    assignParts(trunk, trunk)


def doConnect(name, occurrence_1, occurrence_2):
    if name.startswith('frame_'):
        name = '_'.join(name.split('_')[1:])
        print(f"= Found frame {name}")
        part = assignations[occurrence_1] if config['drawFrames'] else 'frame'
        frame = data['matedEntities'][1]['matedOccurrence']
        frames[occurrence_1].append([name, frame])
        assignParts(occurrence_2, part)
    else:
        connectParts(occurrence_2, assignations[occurrence_1])

# Spreading parts assignations, this parts mainly does two things:
# 1. Finds the parts of the top level assembly that are not directly in a sub assembly and try to assign them
#    to an existing link that was identified before
# 2. Among those parts, finds the ones that are frames (connected with a frame_* connector)
changed = True
while changed:
    changed = False
    for feature in features:
        if feature['featureType'] != 'mate' or feature['suppressed']:
            continue

        data = feature['featureData']

        if len(data['matedEntities']) != 2 \
                or len(data['matedEntities'][0]['matedOccurrence']) == 0 \
                or len(data['matedEntities'][1]['matedOccurrence']) == 0:
            continue

        occurrenceA = data['matedEntities'][0]['matedOccurrence'][0]
        occurrenceB = data['matedEntities'][1]['matedOccurrence'][0]

        if (occurrenceA not in assignations) != (occurrenceB not in assignations):
            changed = True
            name = data['name']
            if occurrenceA in assignations:
                doConnect(name, occurrenceA, occurrenceB)
            else:
                doConnect(name, occurrenceB, occurrenceA)

# Building and checking robot tree, here we:
# 1. Search for robot trunk (which will be the top-level link)
# 2. Scan for orphaned parts (if you add something floating with no mate to anything)
#    that are then assigned to trunk by default
# 3. Collect all the pieces of the robot tree
print("\n" + Style.BRIGHT + '* Building robot tree' + Style.RESET_ALL)

for childId in relations:
    entry = relations[childId]
    if entry['parent'] not in relations:
        trunk = entry['parent']
        break
trunkOccurrence = getOccurrence([trunk])
print(Style.BRIGHT + '* Trunk is ' +
      trunkOccurrence['instance']['name'] + Style.RESET_ALL)

for occurrence in occurrences.values():
    if occurrence['assignation'] is None:
        print(Fore.YELLOW + 'WARNING: part (' +
              occurrence['instance']['name']+') has no assignation, connecting it with trunk' + Style.RESET_ALL)
        child = occurrence['path'][0]
        connectParts(child, trunk)


def collect(part_id):
    part = {}
    part['id'] = part_id
    part['children'] = []
    for childId in relations:
        entry = relations[childId]
        if entry['parent'] == part_id:
            child = collect(childId)
            child['axis_frame'] = entry['worldAxisFrame']
            child['z_axis'] = entry['zAxis']
            child['dof_name'] = entry['name']
            child['jointType'] = entry['type']
            child['jointLimits'] = entry['limits']
            part['children'].append(child)
    return part


tree = collect(trunk)
