import numpy as np

from g1_playground.action import JointAction, Mode
from g1_playground.policies.policy import Policy
from g1_playground.state import G1State
from g1_playground.robot import (
    BODY_NUM_MOTORS,
    G129DofJointIndex,
    G1_29DOF_NUM_MOTORS,
)


class AnkleSwingPolicy(Policy):
    """Three-stage demo policy.

    Stage 1 (0-3s):  ramp from initial pose to zero pose.
    Stage 2 (3-6s):  swing ankles in PR mode (pitch/roll independent).
    Stage 3 (6s+):   swing ankles in AB mode (parallel A/B joints) plus wrist roll.

    Body-only (29 motors) by default. Pass num_motors=43 if running with Dex3 hands;
    the hand portion of q stays at 0, so hands will track zero position with the
    robot's DEFAULT_HAND_KP/KD. Use a different policy if you want hands limp.
    """

    STAGE_DURATION = 3.0
    DT = 0.002  # 100 Hz policy

    def __init__(self, num_motors: int = G1_29DOF_NUM_MOTORS):
        self._num_motors = num_motors
        self._t = 0.0
        self._initial_q = np.zeros(num_motors, dtype=np.float32)

    @property
    def dt(self) -> float:
        return self.DT

    def reset(self, state: G1State) -> None:
        self._t = 0.0
        for i in range(BODY_NUM_MOTORS):
            self._initial_q[i] = state.body.motor_state[i].q

    def step(self, state: G1State) -> JointAction:
        self._t += self.DT
        q = np.zeros(self._num_motors, dtype=np.float32)

        if self._t < self.STAGE_DURATION:
            ratio = float(np.clip(self._t / self.STAGE_DURATION, 0.0, 1.0))
            q[:BODY_NUM_MOTORS] = (1.0 - ratio) * self._initial_q[:BODY_NUM_MOTORS]
            return JointAction(q=q, mode_pr=Mode.PR)

        if self._t < 2 * self.STAGE_DURATION:
            t = self._t - self.STAGE_DURATION
            max_p = np.deg2rad(30.0)
            max_r = np.deg2rad(10.0)
            q[G129DofJointIndex.LeftAnklePitch] = max_p * np.sin(2 * np.pi * t)
            q[G129DofJointIndex.LeftAnkleRoll] = max_r * np.sin(2 * np.pi * t)
            q[G129DofJointIndex.RightAnklePitch] = max_p * np.sin(2 * np.pi * t)
            q[G129DofJointIndex.RightAnkleRoll] = -max_r * np.sin(2 * np.pi * t)
            return JointAction(q=q, mode_pr=Mode.PR)

        t = self._t - 2 * self.STAGE_DURATION
        max_a = np.deg2rad(30.0)
        max_b = np.deg2rad(10.0)
        max_wrist = np.deg2rad(30.0)
        q[G129DofJointIndex.LeftAnkleA] = max_a * np.sin(2 * np.pi * t)
        q[G129DofJointIndex.LeftAnkleB] = max_b * np.sin(2 * np.pi * t + np.pi)
        q[G129DofJointIndex.RightAnkleA] = -max_a * np.sin(2 * np.pi * t)
        q[G129DofJointIndex.RightAnkleB] = -max_b * np.sin(2 * np.pi * t + np.pi)
        q[G129DofJointIndex.LeftWristRoll] = max_wrist * np.sin(2 * np.pi * t)
        q[G129DofJointIndex.RightWristRoll] = max_wrist * np.sin(2 * np.pi * t)
        return JointAction(q=q, mode_pr=Mode.AB)
