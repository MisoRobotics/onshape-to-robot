import math
import os
from abc import ABC, abstractmethod, abstractproperty
from pathlib import Path
from typing import List, Optional

import numpy as np
from lxml import etree

from . import stl_combine
from .material_tags import MaterialTag

XML_ENCODING = "unicode"
XML_INDENT = " " * 2
_INERTIA_ENTRIES = (
    ("ixx", (0, 0)),
    ("ixy", (0, 1)),
    ("ixz", (0, 2)),
    ("iyy", (1, 1)),
    ("iyz", (1, 2)),
    ("izz", (2, 2)),
)


def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def xml_to_string(element: etree._Element) -> str:
    """Convert an XML element to a string."""
    etree.indent(element, space="  ")
    return etree.tostring(element, pretty_print=True, encoding=XML_ENCODING).strip()



def origin(matrix):
    urdf = '<origin xyz="%g %g %g" rpy="%g %g %g" />'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    return urdf % (x, y, z, rpy[0], rpy[1], rpy[2])


def pose(matrix, frame=''):
    sdf = '<pose>%g %g %g %g %g %g</pose>'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    if frame != '':
        sdf = '<frame name="'+frame+'_frame">'+sdf+'</frame>'

    return sdf % (x, y, z, rpy[0], rpy[1], rpy[2])


def gazebo_fixed_joints(name: str) -> str:
    """Preserve fixed joints in Gazebo instead of merging them."""
    gazebo = etree.Element("gazebo", reference=name)
    etree.SubElement(gazebo, "preserveFixedJoint").text = "true"
    etree.SubElement(gazebo, "disabledFixedJointLumping").text = "true"
    return xml_to_string(gazebo)


class RobotDescription(ABC):

    def __init__(self, robotName: str) -> None:
        self.robotName: str = robotName
        self.drawCollisions: bool = False
        self.relative: bool = True
        self.mergeSTLs: str = "no"
        self.mergeSTLsCollisions: bool = False
        self.useFixedLinks: bool = False
        self.simplifySTLs: str = "no"
        self.maxSTLSize: int  = 3
        self.xml: str = ""
        self.jointMaxEffort: float = 1.
        self.jointMaxVelocity: float = 10.
        self.noDynamics: bool = False
        self.packageName: str = ""
        self.packageType: str = ""
        self.addDummyBaseLink: bool = False
        self.output_dir: Optional[Path] = None
        self.use_material_tags: bool = False
        self.gazebo_materials = {}

    @abstractproperty
    def modelFormat(self) -> str:
        """Return the format of the model, i.e., 'urdf' or 'sdf'."""
        raise NotImplementedError()

    @abstractproperty
    def modelFilePath(self) -> str:
        """Return the output filename for the model."""
        raise NotImplementedError()

    @property
    def createRosPackage(self) -> bool:
        """Return True if a ROS package should be created."""
        return self.packageType != "none"

    @abstractmethod
    def getMeshUrl(self, filename: str) -> str:
        """Return the URL for the specified mesh file."""
        raise NotImplementedError()

    @property
    def meshDir(self) -> Path:
        """Return the subdirectory in which meshes should be placed."""
        path = self.getMeshDirHelper()
        path.mkdir(parents=True, exist_ok=True)
        return path

    @abstractmethod
    def getMeshDirHelper(self) -> Path:
        """Return the subdirectory in which meshes should be placed."""
        raise NotImplementedError()

    @property
    def modelDir(self) -> Path:
        """Return the path to the robot model directory."""
        path = self.getModelDirHelper()
        path.mkdir(parents=True, exist_ok=True)
        return path

    @abstractmethod
    def getModelDirHelper(self) -> Path:
        """Return the path to the robot model directory."""
        raise NotImplementedError()

    def shouldMergeSTLs(self, node):
        return self.mergeSTLs == 'all' or self.mergeSTLs == node

    def shouldSimplifySTLs(self, node):
        return self.simplifySTLs == 'all' or self.simplifySTLs == node

    def append(self, str):
        self.xml += str+"\n"

    def jointMaxEffortFor(self, jointName):
        if isinstance(self.jointMaxEffort, dict):
            if jointName in self.jointMaxEffort:
                return self.jointMaxEffort[jointName]
            else:
                return self.jointMaxEffort['default']
        else:
            return self.jointMaxEffort

    def jointMaxVelocityFor(self, jointName):
        if isinstance(self.jointMaxVelocity, dict):
            if jointName in self.jointMaxVelocity:
                return self.jointMaxVelocity[jointName]
            else:
                return self.jointMaxVelocity['default']
        else:
            return self.jointMaxVelocity

    def resetLink(self):
        self._mesh = {'visual': None, 'collision': None}
        self._color = np.array([0., 0., 0., 0.])
        self._color_mass = 0
        self._link_childs = 0
        self._visuals = []
        self._dynamics = []

    @abstractmethod
    def append_inertial(
        self,
        mass: float = None,
        com: Optional[List[float]] = None,
        inertia: Optional[List[List[float]]]= None,
        frame: Optional[str] = None,
    ) -> None:
        """Append an <inertial> element."""
        raise NotImplementedError()

    @abstractmethod
    def append_material(self, name: str, color: List[float]) -> None:
        """Append a <material> element."""
        raise NotImplementedError()

    def addLinkDynamics(self, matrix, mass, com, inertia):
        # Inertia
        I = np.matrix(np.reshape(inertia[:9], (3, 3)))
        R = matrix[:3, :3]
        # Expressing COM in the link frame
        com = np.array(
            (matrix*np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
        # Expressing inertia in the link frame
        inertia = R.T*I*R

        self._dynamics.append({
            'mass': mass,
            'com': com,
            'inertia': inertia
        })

    def addFrame(self, name, matrix):
        print(f"- Adding frame {name} to robot")
        self.addDummyLink(name)

        # Linking it with last link with a fixed link
        self.addFixedJoint(self._link_name, name, matrix, name+'_frame')

    @abstractmethod
    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        raise NotImplementedError()

    @abstractmethod
    def addFixedJoint(self, parent, child, matrix, name=None):
        raise NotImplementedError()

    def mergeSTL(self, stl, matrix, color, mass, node='visual'):
        if node == 'visual':
            self._color += np.array(color) * mass
            self._color_mass += mass

        m = stl_combine.load_mesh(stl)
        stl_combine.apply_matrix(m, matrix)

        if self._mesh[node] is None:
            self._mesh[node] = m
        else:
            self._mesh[node] = stl_combine.combine_meshes(self._mesh[node], m)

    def linkDynamics(self):
        mass = 0
        com = np.array([0.0]*3)
        inertia = np.matrix(np.zeros((3, 3)))
        identity = np.matrix(np.eye(3))

        for dynamic in self._dynamics:
            mass += dynamic['mass']
            com += dynamic['com']*dynamic['mass']

        if mass > 0:
            com /= mass

        # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=246
        for dynamic in self._dynamics:
            r = dynamic['com'] - com
            p = np.matrix(r)
            inertia += dynamic['inertia'] + \
                (np.dot(r, r)*identity - p.T*p)*dynamic['mass']

        return mass, com, inertia

    def write(self, filepath: Path) -> None:
        """Write the robot description to the passed filepath.

        Args:
            filepath: The path to the robot description to write.

        """
        print("Writing robot description to {}".format(filepath))
        with open(filepath, 'w', encoding="utf-8") as stream:
            stream.write(self.xml)
        # xml = BeautifulSoup(self.xml, "xml").prettify()
        # element = ElementTree.fromstring(xml)
        # tree = ElementTree.ElementTree(element)
        # tree.write(filepath, encoding="utf-8")


class RobotURDF(RobotDescription):
    def __init__(self, name):
        super().__init__(name)
        self.append('<robot name="' + self.robotName + '">')
        pass

    @property
    def modelFormat(self) -> str:
        """Return the format of the model."""
        return "urdf"

    def getModelDirHelper(self) -> Path:
        """Return the path to the robot model directory."""
        model_dir = self.output_dir
        if self.createRosPackage:
            model_dir /= "urdf"
        return model_dir

    def getMeshUrl(self, filename: str) -> str:
        """Return the URL for the specified mesh file."""
        path = f"package://{self.packageName}"
        if self.createRosPackage:
            path = f"{path}/meshes"
        path = f"{path}/{filename}"
        return path

    def getMeshDirHelper(self) -> Path:
        """Return the subdirectory in which meshes should be placed."""
        mesh_dir = self.output_dir
        if self.createRosPackage:
            mesh_dir /= "meshes"
        return mesh_dir

    @property
    def modelFilePath(self) -> Path:
        """Return the output filename for the model."""
        return self.modelDir / "robot.urdf"

    def append_inertial(
        self,
        mass: float,
        com: Optional[List[float]] = None,
        inertia: Optional[np.ndarray]= None,
        frame: Optional[str] = None,
    ) -> None:
        """Append an <inertial> element."""
        inertial = etree.Element("inertial")
        if com is not None:
            xyz = f"{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}"
            origin = etree.SubElement(inertial, "origin", xyz=xyz)
        etree.SubElement(inertial, "mass", value=f"{mass:.5f}")
        if inertia is None:
            inertia = np.zeros((3, 3))
        kwargs = {e: f"{inertia[r, c]:.5f}" for e, (r, c) in _INERTIA_ENTRIES}
        etree.SubElement(inertial, "inertia", **kwargs)
        self.append(xml_to_string(inertial))

    def append_material(self, name: str, color: List[float]) -> None:
        """Append a <material> element."""
        material = etree.Element("material", name=f"{name}_material")
        rgba = " ".join([f"{c:.3f}" for c in color])
        etree.SubElement(material, "color", rgba=rgba)
        script = etree.SubElement(material, "script")
        etree.SubElement(script, "name").text = f"{self.robotName}/{name}"
        if name in self.gazebo_materials:
            print(f"Material name {name} already exists.")
        self.gazebo_materials[name] = color
        self.append(xml_to_string(material))

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        self.append(f'<link name="{name}">')
        mass = 0 if self.noDynamics else 1e-4
        self.append_inertial(mass)
        if visualSTL is not None:
            self.addSTL(
                visualMatrix, visualSTL, visualColor, f"{name}_visual", "visual"
            )
        self.append('</link>')

    def addDummyBaseLinkMethod(self, name):
        # adds a dummy base_link for ROS users
        self.append('<link name="world"></link>')
        self.append('<joint name="fixed" type="fixed">')
        self.append('  <parent link="world"/>')
        self.append('  <child link="' + name + '" />')
        self.append('</joint>')

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent+'_'+child+'_fixing'

        self.append('<joint name="'+name+'" type="fixed">')
        self.append(origin(matrix))
        self.append('  <parent link="'+parent+'" />')
        self.append('  <child link="'+child+'" />')
        self.append('</joint>')
        self.append(gazebo_fixed_joints(name))

    def startLink(self, name, matrix):
        self._link_name = name
        self.resetLink()

        if self.addDummyBaseLink:
            self.addDummyBaseLinkMethod(name)
            self.addDummyBaseLink = False
        self.append('<link name="'+name+'">')


    def endLink(self):
        mass, com, inertia = self.linkDynamics()

        for node in ['visual', 'collision']:
            if self._mesh[node] is not None:
                if node == 'visual' and self._color_mass > 0:
                    color = self._color / self._color_mass
                else:
                    color = [0.5, 0.5, 0.5, 1.0]

                filename = self._link_name+'_'+node+'.stl'
                stl_combine.save_mesh(
                    self._mesh[node], self.meshDir+'/'+filename)
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(self.meshDir+'/'+filename, self.maxSTLSize)
                self.addSTL(np.identity(4), filename, color, self._link_name, node)

        self.append_inertial(mass, com, inertia)
        if self.useFixedLinks:
            self.append(
                '<visual><geometry><box size="0 0 0" /></geometry></visual>')

        self.append('</link>')
        self.append(gazebo_fixed_joints(self._link_name))
        self.append('')

        if self.useFixedLinks:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = '%s_%d' % (self._link_name, n)
                self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
                self.addJoint('fixed', self._link_name, visual_name,
                              np.eye(4), visual_name+'_fixing', None)

    def addSTL(self, matrix, stl, color, name, node='visual'):
        self.append('<'+node+'>')
        self.append(origin(matrix))
        self.append('<geometry>')
        self.append(f'<mesh filename="{self.getMeshUrl(stl)}"/>')
        self.append('</geometry>')
        if node == "visual":
            self.append_material(name, color)
        self.append('</'+node+'>')

    def addPart(self, matrix, stl, material_tag: MaterialTag, mass, com, inertia, color, shapes=None, name=''):
        if stl is not None:
            if not self.drawCollisions:
                if self.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.packageName + os.path.basename(stl), color])
                elif self.shouldMergeSTLs('visual'):
                    self.mergeSTL(stl, matrix, color, mass)
                else:
                    self.addSTL(
                        matrix, os.path.basename(stl), color, name, 'visual')

            entries = ['collision'] if self.use_material_tags and material_tag else []
            if self.drawCollisions or material_tag.also_visual:
                entries.append('visual')
            for entry in entries:
                if shapes is None:
                    # We don't have pure shape, we use the mesh
                    if self.shouldMergeSTLs(entry):
                        self.mergeSTL(stl, matrix, color, mass, entry)
                    else:
                        self.addSTL(matrix, os.path.basename(
                            stl), color, name, entry)
                else:
                    # Inserting pure shapes in the URDF model
                    self.append('<!-- Shapes for '+name+' -->')
                    for shape in shapes:
                        self.append('<'+entry+'>')
                        self.append(origin(matrix*shape['transform']))
                        self.append('<geometry>')
                        if shape['type'] == 'cube':
                            self.append('<box size="%g %g %g" />' %
                                        tuple(shape['parameters']))
                        if shape['type'] == 'cylinder':
                            self.append(
                                '<cylinder length="%g" radius="%g" />' % tuple(shape['parameters']))
                        if shape['type'] == 'sphere':
                            self.append('<sphere radius="%g" />' %
                                        shape['parameters'])
                        self.append('</geometry>')

                        if entry == 'visual':
                            self.append_material(name, color)
                            self.gazebo_materials[name] = color
                        self.append('</'+entry+'>')

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
        self.append('<joint name="'+name+'" type="'+jointType+'">')
        self.append(origin(transform))
        self.append('<parent link="'+linkFrom+'" />')
        self.append('<child link="'+linkTo+'" />')
        self.append('<axis xyz="%g %g %g"/>' % tuple(zAxis))
        lowerUpperLimits = ''
        if jointLimits is not None:
            lowerUpperLimits = 'lower="%g" upper="%g"' % jointLimits
        self.append('<limit effort="%g" velocity="%g" %s/>' %
                    (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
        self.append('<joint_properties friction="0.0"/>')
        self.append('</joint>')
        self.append('')

    def finalize(self):
        self.append(self.additionalXML)
        self.append('</robot>')


class RobotSDF(RobotDescription):
    def __init__(self, name):
        super().__init__(name)
        self.relative = False
        self.append('<sdf version="1.6">')
        self.append('<model name="'+self.robotName + '">')

    @property
    def modelFormat(self) -> str:
        """Return the format of the model."""
        return "sdf"

    @property
    def modelFilePath(self) -> Path:
        """Return the output filename for the model."""
        return self.modelDir / "model.sdf"

    def getModelDirHelper(self) -> Path:
        """Return the path to the robot model directory."""
        model_dir = self.output_dir
        if self.createRosPackage:
            model_dir /= "models" / self.robotName
        return model_dir

    def getMeshUrl(self, filename: str) -> str:
        """Return the URL for the specified mesh file."""
        return f"meshes/{filename}"

    def getMeshDirHelper(self) -> Path:
        """Return the subdirectory in which meshes should be placed."""
        mesh_dir = self.modelDir
        if self.createRosPackage:
            mesh_dir /= "meshes"
        return mesh_dir

    def append_inertial(
        self,
        mass: float,
        com: Optional[List[float]] = None,
        inertia: Optional[List[List[float]]]= None,
        frame: Optional[str] = None,
    ) -> None:
        """Create an <inertial> element."""
        inertial = etree.Element("inertial")
        if com is not None:
            kwargs = {"frame": frame} if frame else {}
            pose = etree.SubElement(inertial, "pose", **kwargs)
            pose.text = f"{com[0]:.5f} {com[1]:.5f} {com[2]:.5f} 0 0 0"
        etree.SubElement(inertial, "mass").text = f"{mass:.5f}"
        if inertia is None:
            inertia = np.zeros((3, 3))
        inertia_element = etree.SubElement(inertial, "inertia")
        for element, (c, r) in _INERTIA_ENTRIES:
            etree.SubElement(inertia_element, element).text = f"{inertia[r][c]:.5f}"
        self.append(xml_to_string(inertial))

    def append_material(self, name: str, color: List[float]) -> None:
        """Append a <material> element."""
        material = etree.Element("material", name=f"{name}_material")
        rgba = " ".join([f"{c:.3f}" for c in color])
        etree.SubElement(material, "ambient").text = rgba
        etree.SubElement(material, "diffuse").text = rgba
        script = etree.SubElement(material, "script")
        etree.SubElement(script, "name").text = f"{self.robotName}/{name}"
        if name in self.gazebo_materials:
            print(f"Material name {name} already exists.")
        self.gazebo_materials[name] = color
        self.append(xml_to_string(material))

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = f"{parent}_{child}_fixing"

        self.append(f'<joint name="{name}" type="fixed">')
        self.append(pose(matrix))
        self.append(f'<parent>parent</parent>')
        self.append(f'<child>child</child>')
        self.append('</joint>')
        self.append(gazebo_fixed_joints(name))
        self.append('')

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        self.append(f'<link name="{name}">')
        self.append_inertial(mass=1e-4)
        if visualSTL is not None:
            self.addSTL(
                visualMatrix, visualSTL, visualColor, f"{name}_visual", "visual"
            )
        self.append('</link>')

    def startLink(self, name, matrix):
        self._link_name = name
        self.resetLink()
        self.append('<link name="'+name+'">')
        self.append(pose(matrix, name))

    def endLink(self):
        mass, com, inertia = self.linkDynamics()

        for node in ['visual', 'collision']:
            if self._mesh[node] is not None:
                color = self._color / self._color_mass
                filename = self._link_name+'_'+node+'.stl'
                stl_combine.save_mesh(
                    self._mesh[node], self.meshDir / filename)
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(self.meshDir / filename, self.maxSTLSize)
                self.addSTL(np.identity(4), self.meshDir / filename, color, self._link_name, 'visual')

        self.append_inertial(mass, com, inertia, f"{self._link_name}_frame")
        if self.useFixedLinks:
            self.append(
                '<visual><geometry><box><size>0 0 0</size></box></geometry></visual>')

        self.append(gazebo_fixed_joints(self._link_name))
        self.append('</link>')
        self.append('')

        if self.useFixedLinks:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = '%s_%d' % (self._link_name, n)
                self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
                self.addJoint('fixed', self._link_name, visual_name,
                              np.eye(4), f"{visual_name}_fixing", None)

    def addSTL(self, matrix, stl, color, name, node='visual'):
        self.append('<'+node+' name="'+name+'_visual">')
        self.append(pose(matrix))
        self.append('<geometry>')
        self.append(f'<mesh><uri>{stl}</uri></mesh>')
        self.append('</geometry>')
        if node == 'visual':
            self.append_material(name, color)
        self.append('</'+node+'>')

    def addPart(self, matrix, stl, material_tag: MaterialTag, mass, com, inertia, color, shapes=None, name=''):
        name = self._link_name+'_'+str(self._link_childs)+'_'+name
        self._link_childs += 1

        # self.append('<link name="'+name+'">')
        # self.append(pose(matrix))

        if stl is not None:
            if not self.drawCollisions:
                if self.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.meshDir / os.path.basename(stl), color])
                elif self.shouldMergeSTLs('visual'):
                    self.mergeSTL(stl, matrix, color, mass)
                else:
                    self.addSTL(matrix, os.path.basename(
                        stl), color, name, 'visual')

            entries = ['collision'] if self.use_material_tags and material_tag else []
            if self.drawCollisions or material_tag.also_visual:
                    entries.append('visual')
            for entry in entries:
                if shapes is None:
                    # We don't have pure shape, we use the mesh
                    if self.shouldMergeSTLs(entry):
                        self.mergeSTL(stl, matrix, color, mass, entry)
                    else:
                        self.addSTL(matrix, stl, color, name, entry)
                else:
                    # Inserting pure shapes in the URDF model
                    k = 0
                    self.append('<!-- Shapes for '+name+' -->')
                    for shape in shapes:
                        k += 1
                        self.append('<'+entry+' name="'+name +
                                    '_'+entry+'_'+str(k)+'">')
                        self.append(pose(matrix*shape['transform']))
                        self.append('<geometry>')
                        if shape['type'] == 'cube':
                            self.append('<box><size>%g %g %g</size></box>' %
                                        tuple(shape['parameters']))
                        if shape['type'] == 'cylinder':
                            self.append(
                                '<cylinder><length>%g</length><radius>%g</radius></cylinder>' % tuple(shape['parameters']))
                        if shape['type'] == 'sphere':
                            self.append(
                                '<sphere><radius>%g</radius></sphere>' % shape['parameters'])
                        self.append('</geometry>')

                        if entry == 'visual':
                            self.append_material(name, color)
                        self.append('</'+entry+'>')

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
        self.append('<joint name="'+name+'" type="'+jointType+'">')
        self.append(pose(transform))
        self.append('<parent>'+linkFrom+'</parent>')
        self.append('<child>'+linkTo+'</child>')
        self.append('<axis>')
        self.append('<xyz>%g %g %g</xyz>' % tuple(zAxis))
        lowerUpperLimits = ''
        if jointLimits is not None:
            lowerUpperLimits = '<lower>%g</lower><upper>%g</upper>' % jointLimits
        self.append('<limit><effort>%g</effort><velocity>%g</velocity>%s</limit>' %
                    (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
        self.append('</axis>')
        self.append('</joint>')
        self.append('')
        # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))

    def finalize(self):
        self.append(self.additionalXML)
        self.append('</model>')
        self.append('</sdf>')
