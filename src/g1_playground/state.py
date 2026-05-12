from dataclasses import dataclass

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, HandState_


@dataclass
class G1State:
    """Snapshot of robot state passed to the policy each step.

    left_hand and right_hand are None when the robot is configured without Dex3 hands.
    """
    body: LowState_
    left_hand: HandState_ | None = None
    right_hand: HandState_ | None = None