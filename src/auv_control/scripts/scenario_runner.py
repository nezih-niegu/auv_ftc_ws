#!/usr/bin/env python3
# ============================================================================
#  scenario_runner.py
#
#  Reproduces one of the four fault scenarios from Zhang et al., Sensors 2024,
#  24, 3029 (Figures 6-11). Runs for 300 s of simulation time and injects
#  the configured fault at t = 150 s, matching the paper exactly.
#
#  Usage:
#      ros2 run auv_control scenario_runner.py \
#          --ros-args -p scenario:=fig6_abrupt_thrust
#
#  Scenarios (matching the paper):
#      fig6_abrupt_thrust  : abrupt u1 loss @150 s       (Fig 6 top)
#      fig6_slow_thrust    : u1 degraded linearly 150-250 s -> 0   (Fig 6 bottom)
#      fig7_abrupt_moment  : abrupt u4 loss @150 s       (Fig 7 top)
#      fig7_slow_moment    : u4 degraded linearly 150-250 s -> 0   (Fig 7 bottom)
# ============================================================================
import rclpy
from rclpy.node import Node

from auv_control.srv import InjectFault


SCENARIOS = {
    # scenario name          : (thruster id, target factor, fault type, ramp)
    "fig6_abrupt_thrust"    : (1, 0.0, "abrupt", 0.0),
    "fig6_slow_thrust"      : (1, 0.0, "slow",   100.0),
    "fig7_abrupt_moment"    : (4, 0.0, "abrupt", 0.0),
    "fig7_slow_moment"      : (4, 0.0, "slow",   100.0),
}

FAULT_TIME_S = 150.0
END_TIME_S   = 300.0


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")
        self.declare_parameter("scenario", "fig6_abrupt_thrust")
        name = self.get_parameter("scenario").get_parameter_value().string_value

        if name not in SCENARIOS:
            self.get_logger().error(
                f"unknown scenario '{name}'. valid: {list(SCENARIOS.keys())}")
            rclpy.shutdown()
            return

        self._id, self._factor, self._type, self._ramp = SCENARIOS[name]
        self._name      = name
        self._injected  = False
        self._shutting_down = False
        self._t0        = self.get_clock().now().nanoseconds * 1e-9

        self._cli = self.create_client(InjectFault, "/auv/inject_fault")
        self.get_logger().info(
            f"scenario '{name}': will inject u{self._id} -> "
            f"{self._factor} ({self._type}) at t=+{FAULT_TIME_S:.0f}s, "
            f"end at t=+{END_TIME_S:.0f}s")

        # Poll at 2 Hz.
        self._timer = self.create_timer(0.5, self._tick)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._t0

        if not self._injected and elapsed >= FAULT_TIME_S:
            if not self._cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warning(
                    "/auv/inject_fault not yet available — retrying")
                return
            req = InjectFault.Request()
            req.thruster_id  = self._id
            req.fault_factor = self._factor
            req.fault_type   = self._type
            req.ramp_seconds = self._ramp
            self._cli.call_async(req)
            self._injected = True
            self.get_logger().warning(
                f"[t=+{elapsed:.1f}s] FAULT INJECTED for scenario '{self._name}'")

        if not self._shutting_down and elapsed >= END_TIME_S:
            self.get_logger().info(
                f"[t=+{elapsed:.1f}s] scenario '{self._name}' complete — exiting")
            self._shutting_down = True
            # Stop our timer and let rclpy.spin() return on shutdown.
            self._timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
