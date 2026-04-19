"""Terrain safety interpretation."""
from typing import Any, Dict


# This set marks surfaces that should trigger a rover-to-drone request.
DANGEROUS_TERRAINS = {"ditch", "cliff", "water", "steep", "ravine", "drop"}

# This set marks surfaces that are acceptable for drone landing.
LANDING_SAFE_TERRAINS = {"clear", "flat", "grass", "road", "trail", "pad"}


# This helper converts a terrain label into safety flags used by the engine.
def get_terrain_warning(
    state: Dict[str, Any],
    manual_warning: bool | None = None,
) -> Dict[str, Any]:
    terrain_type = str(state.get("terrain", "clear")).strip().lower()

    # A manual override lets ROS or UI code force a terrain warning.
    if manual_warning is not None:
        terrain_warning = manual_warning
        source = "manual_override"
    else:
        terrain_warning = terrain_type in DANGEROUS_TERRAINS
        source = "parsed_terrain"

    # Landing is only safe on known-good surfaces with no active warning.
    landing_safe = terrain_type in LANDING_SAFE_TERRAINS and not terrain_warning

    # A normalized terrain payload keeps downstream logic simple.
    return {
        "terrain_warning": terrain_warning,
        "terrain_type": terrain_type,
        "landing_safe": landing_safe,
        "source": source,
    }
