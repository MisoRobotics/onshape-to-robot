import dataclasses
import math
from typing import (
    Final,
    List,
    Optional,
    Tuple,
)

from colorama import Back, Fore, Style
from onshape_client import Client as OnshapeClient
from onshape_client import OnshapeElement

from .onshape_api.client import Client

configuration_parameters = {}


def init(client, config, root, workspaceId, assemblyId):
    global configuration_parameters

    # Retrieving root configuration parameters
    configuration_parameters = {}
    parts = root["fullConfiguration"].split(";")
    for part in parts:
        kv = part.split("=")
        if len(kv) == 2:
            configuration_parameters[kv[0]] = kv[1].replace("+", " ")


def readExpression(expression):
    # Expression can itself be a variable from configuration
    # XXX: This doesn't handle all expression, only values and variables
    if expression[0] == "#":
        expression = configuration_parameters[expression[1:]]
    if expression[0:2] == "-#":
        expression = "-" + configuration_parameters[expression[2:]]

    parts = expression.split(" ")

    # Checking the unit, returning only radians and meters
    if parts[1] == "deg":
        return math.radians(float(parts[0]))
    elif parts[1] in ["radian", "rad"]:
        return float(parts[0])
    elif parts[1] == "mm":
        return float(parts[0]) / 1000.0
    elif parts[1] == "m":
        return float(parts[0])
    elif parts[1] == "in":
        return float(parts[0]) * 0.0254
    else:
        print(Fore.RED + "Unknown unit: " + parts[1] + Style.RESET_ALL)
        exit()


def readParameterValue(parameter, name):
    # This is an expression
    if parameter["typeName"] == "BTMParameterNullableQuantity":
        return readExpression(parameter["message"]["expression"])
    if parameter["typeName"] == "BTMParameterConfigured":
        message = parameter["message"]
        parameterValue = configuration_parameters[message["configurationParameterId"]]

        for value in message["values"]:
            if value["typeName"] == "BTMConfiguredValueByBoolean":
                booleanValue = parameterValue == "true"
                if value["message"]["booleanValue"] == booleanValue:
                    return readExpression(
                        value["message"]["value"]["message"]["expression"]
                    )
            elif value["typeName"] == "BTMConfiguredValueByEnum":
                if value["message"]["enumValue"] == parameterValue:
                    return readExpression(
                        value["message"]["value"]["message"]["expression"]
                    )
            else:
                print(
                    Fore.RED
                    + "Can't read value of parameter "
                    + name
                    + " configured with "
                    + value["typeName"]
                    + Style.RESET_ALL
                )
                exit()

        print(Fore.RED + "Could not find the value for " + name + Style.RESET_ALL)
    else:
        print(
            Fore.RED
            + "Unknown feature type for "
            + name
            + ": "
            + parameter["typeName"]
            + Style.RESET_ALL
        )
        exit()


# Gets the limits of a given joint


@dataclasses.dataclass
class FeatureSource:
    """Fetch feature data from the Onshape API."""

    def __init__(self, client: OnshapeClient, element: OnshapeElement) -> None:
        self._client: Final[OnshapeClient] = client
        self._element: Final[OnshapeElement] = element

    def get_feature(
        self, name: str, has_parameters: Optional[List[str]] = None
    ) -> dict:
        features = self._client.assemblies_api.get_features(
            self._element.did, self._element.wvm, self._element.wvmid, self._element.eid
        )
        for feature in features["features"]:
            if feature["name"] == name:
                if has_parameters is None:
                    return feature
                if all(
                    any(key == p.parameter_id for p in feature["parameters"])
                    for key in has_parameters
                ):
                    return feature
        raise Exception(f"Feature '{name}' not found.")

    def get_limits(self, name: str) -> Optional[Tuple[float, float]]:
        """Return the joint limits from the named joint on the specified assembly."""
        feature = self.get_feature(name, ["mateType"])
        parameters = {p.parameter_id: p for p in feature.parameters}
        mate_type = parameters["mateType"].value
        enabled = parameters["limitsEnabled"].value
        if mate_type == "REVOLUTE":
            joint_type = "revolute"
            infix = "Axial"
        elif mate_type == "SLIDER":
            joint_type = "prismatic"
            infix = ""
        else:
            raise ValueError(f"Unknown joint type '{joint_type}'")

        if not enabled:
            print(
                f"{Fore.YELLOW}WARNING: Joint '{name}' of type '{joint_type}' has no limits.{Style.RESET_ALL}"
            )
            return None

        return tuple(
            (
                readExpression(parameters[f"limit{infix}Z{suffix}"].expression)
                for suffix in ("Min", "Max")
            )
        )
