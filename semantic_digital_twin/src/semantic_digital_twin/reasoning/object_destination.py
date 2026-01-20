from typing import List, Type

from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticAnnotation,
    Milk,
    Banana,
    Apple,
    Fridge,
    GarbageBin,
)

DESTINATION_RULES: dict[type[SemanticAnnotation], type[SemanticAnnotation]] = {
    Milk: Fridge,
    Banana: GarbageBin,
    Apple: GarbageBin,
}

def query_object_destination(
        world: World,
        obj: SemanticAnnotation,
) -> List[SemanticAnnotation]:
    """
    Query the knowledge base for suitable destination objects for a given object.

    :param world: The world containing semantic annotations.
    :param obj: The object that should be placed somewhere.
    :return: A list of suitable destination semantic annotations.
    """

    target_type = DESTINATION_RULES.get(type(obj))
    if target_type is None:
        return []

    return world.get_semantic_annotations_by_type(target_type)