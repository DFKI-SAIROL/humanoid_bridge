"""Unitree's motion switcher, faked: which mode holds /lowcmd. No ROS.

Answers unitree_sdk2py's MotionSwitcherClient, whose requests arrive on rt/api/motion_switcher/request.
"""

import json

CHECK_MODE, SELECT_MODE, RELEASE_MODE = 1001, 1002, 1003  # motion_switcher_api.py
OK = 0
NOT_IMPLEMENTED = 3203  # RPC_ERR_SERVER_API_NOT_IMPL
BAD_PARAMETER = 3204  # RPC_ERR_SERVER_API_PARAMETER
BOOT_MODE = "ai"  # the mode a booted G1 holds /lowcmd in


class MotionSwitcher:
    """mode "": released, nothing holds /lowcmd."""

    def __init__(self, mode=""):
        self.mode = mode

    def call(self, api_id, parameter):
        """(code, data) for one request, as the robot answers it."""
        if api_id == CHECK_MODE:
            return OK, json.dumps({"form": "0", "name": self.mode})
        if api_id == RELEASE_MODE:
            self.mode = ""
            return OK, ""
        if api_id == SELECT_MODE:
            try:
                name = json.loads(parameter or "{}").get("name", "")
            except (ValueError, AttributeError):
                name = ""
            if not name:
                return BAD_PARAMETER, ""
            self.mode = name
            return OK, ""
        return NOT_IMPLEMENTED, ""
