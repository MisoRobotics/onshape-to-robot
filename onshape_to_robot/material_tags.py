"""Use Onshape materials to tag collision objects."""
import json
from dataclasses import dataclass
from typing import (
    List,
    Optional,
)

import onshape_client
from dataclasses_json import (
    LetterCase,
    dataclass_json,
)
from onshape_client.oas.models.bt_part_metadata_info import BTPartMetadataInfo


@dataclass_json(letter_case=LetterCase.CAMEL)
@dataclass()
class MaterialTag:
    """Tag this material to use as a collision object."""

    library_name: str
    material_name: str
    also_visual: bool


def load_material_tags(material_tags: List[dict]) -> List[MaterialTag]:
    """Load a list of material tags."""
    return [MaterialTag.from_json(json.dumps(t)) for t in material_tags]


def get_material_tag(
    part: BTPartMetadataInfo, material_tags: List[dict]
) -> Optional[MaterialTag]:
    """Return a MaterialTag corresponding to the passed material."""
    try:
        material = part.material
        for tag in material_tags:
            if (
                material.library_name == tag.library_name
                and material.display_name == tag.material_name
            ):
                return tag
    except onshape_client.oas.exceptions.ApiKeyError:
        pass
    return None
