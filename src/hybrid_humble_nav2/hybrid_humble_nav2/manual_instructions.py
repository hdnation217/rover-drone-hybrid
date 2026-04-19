"""Human-readable key mappings for the local demo harness."""
from .config import AutonomyMode


# This helper prints startup controls for the selected autonomy mode.
def get_startup_instructions(mode: AutonomyMode) -> str:
    if mode == AutonomyMode.MANUAL:
        return (
            "Manual mode:\n"
            "  Rover: W forward | A left | S backward | D right\n"
            "  Drone: W forward | A left | S backward | D right | > ascend | < descend | L land\n"
            "  R switches rover/drone mode | Q quits\n"
        )

    if mode == AutonomyMode.SEMI_AUTONOMOUS:
        return (
            "Semi-autonomous mode:\n"
            "  AI handles movement.\n"
            "  D approves rover -> drone\n"
            "  L approves drone landing\n"
            "  R approves drone -> rover\n"
            "  Q quits\n"
        )

    return (
        "Fully autonomous mode:\n"
        "  AI handles all movement, switching, and landing.\n"
        "  Q quits\n"
    )


# This helper gives a one-line overlay summary for local OpenCV demos.
def get_overlay_controls_line(mode: AutonomyMode, vehicle_mode: str) -> str:
    if mode == AutonomyMode.MANUAL:
        if vehicle_mode == "ROVER":
            return "q quit | r switch mode | w/a/s/d rover move"
        return "q quit | r switch mode | w/a/s/d drone move | </> height | l land"

    if mode == AutonomyMode.SEMI_AUTONOMOUS:
        if vehicle_mode == "ROVER":
            return "q quit | d approve drone switch | t terrain"
        return "q quit | l approve landing | r approve rover switch | t terrain"

    return "q quit | auto mode active | t terrain"
