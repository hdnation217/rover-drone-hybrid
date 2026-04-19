"""Autonomy-policy gating for hybrid actions."""
from typing import Any, Dict

from .actions import CRITICAL_ACTIONS
from .config import AutonomyMode


# This helper applies the current autonomy policy to a proposed action.
def apply_autonomy_mode(result: Dict[str, Any], autonomy_mode: AutonomyMode) -> Dict[str, Any]:
    updated = dict(result)
    action = updated.get("current_action")
    confirmed = updated.get("confirmed", False)
    confirmation_required = updated.get("confirmation_required", False)

    # Manual mode never lets the AI directly execute vehicle actions.
    if autonomy_mode == AutonomyMode.MANUAL:
        updated["execution_allowed"] = False
        updated["human_approval_required"] = False
        updated["execution_reason"] = (
            "Manual mode is active. AI suggestions are shown, but the human controls all actions."
        )
        return updated

    # Semi-autonomous mode only blocks critical actions until approved.
    if autonomy_mode == AutonomyMode.SEMI_AUTONOMOUS:
        if action in CRITICAL_ACTIONS:
            if confirmed:
                updated["execution_allowed"] = True
                updated["human_approval_required"] = False
                updated["execution_reason"] = (
                    "Semi-autonomous mode is active. Human-approved critical action may execute."
                )
            else:
                updated["execution_allowed"] = False
                updated["human_approval_required"] = True
                updated["execution_reason"] = (
                    "Semi-autonomous mode is active. Human approval is required for switching or landing."
                )
            return updated

        # Semi-autonomous mode allows routine motion to proceed automatically.
        updated["execution_allowed"] = True
        updated["human_approval_required"] = confirmation_required
        if confirmation_required:
            updated["execution_reason"] = (
                "Semi-autonomous mode is active. AI may keep moving while waiting for the human decision."
            )
        else:
            updated["execution_reason"] = (
                "Semi-autonomous mode is active. Normal movement may execute automatically."
            )
        return updated

    # Fully autonomous mode allows every proposed action to execute.
    if autonomy_mode == AutonomyMode.FULLY_AUTONOMOUS:
        updated["execution_allowed"] = True
        updated["human_approval_required"] = False
        updated["execution_reason"] = (
            "Fully autonomous mode is active. Action may execute automatically."
        )
        return updated

    # A bad mode value should fail loudly during testing and integration.
    raise ValueError(f"Unsupported autonomy mode: {autonomy_mode}")
