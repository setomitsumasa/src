#!/usr/bin/env python3
"""Fail-closed velocity gate for ERC autonomous navigation.

Nav2 publishes to ``/erc/nav_cmd_vel``. This node is the only ERC path that forwards
commands to the rover's existing ``/cmd_vel`` consumer. It publishes zero velocity
unless localization is initialized, recent GICP fitness is acceptable, the command is
fresh, and no emergency stop is active.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


def gate_reason(*, emergency_stop, initialized, fitness, fitness_age,
                fitness_max, fitness_timeout, command_age, command_timeout,
                require_fitness=True):
    """Return ``open`` or the first fail-closed reason."""
    if emergency_stop:
        return 'emergency_stop'
    if not initialized:
        return 'localization_not_initialized'
    if require_fitness:
        if fitness is None or not math.isfinite(fitness):
            return 'fitness_missing'
        if fitness_age > fitness_timeout:
            return 'fitness_stale'
        if fitness > fitness_max:
            return 'fitness_bad'
    if command_age > command_timeout:
        return 'command_timeout'
    return 'open'


class ErcCmdVelGate(Node):
    def __init__(self):
        super().__init__('erc_cmd_vel_gate')
        input_topic = self.declare_parameter(
            'input_topic', '/erc/nav_cmd_vel').value
        output_topic = self.declare_parameter('output_topic', '/cmd_vel').value
        self.command_timeout = float(
            self.declare_parameter('command_timeout_sec', 0.30).value)
        self.require_fitness = bool(
            self.declare_parameter('require_fitness', True).value)
        self.fitness_max = float(
            self.declare_parameter('fitness_max', 0.15).value)
        self.fitness_timeout = float(
            self.declare_parameter('fitness_timeout_sec', 3.0).value)
        output_rate = float(self.declare_parameter('output_rate', 20.0).value)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self._on_command, 10)
        self.init_sub = self.create_subscription(
            Bool, '/erc/localization_initialized', self._on_initialized, latched)
        self.fitness_sub = self.create_subscription(
            Float32, '/erc/localization_fitness', self._on_fitness, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/erc/emergency_stop', self._on_estop, latched)

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.open_pub = self.create_publisher(
            Bool, '/erc/cmd_vel_gate_open', latched)
        self.state_pub = self.create_publisher(
            String, '/erc/cmd_vel_gate_state', latched)

        self.last_command = Twist()
        self.last_command_time = None
        self.last_fitness = None
        self.last_fitness_time = None
        self.initialized = False
        self.emergency_stop = False
        self.last_reason = None

        self.timer = self.create_timer(
            1.0 / max(1.0, output_rate), self._tick)
        self.get_logger().info(
            f'ERC cmd_vel gate: {input_topic} -> {output_topic}; '
            f'cmd timeout={self.command_timeout:.2f}s, '
            f'fitness<={self.fitness_max:.3f} age<={self.fitness_timeout:.1f}s')

    def _on_command(self, msg):
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    def _on_initialized(self, msg):
        self.initialized = bool(msg.data)

    def _on_fitness(self, msg):
        self.last_fitness = float(msg.data)
        self.last_fitness_time = self.get_clock().now()

    def _on_estop(self, msg):
        self.emergency_stop = bool(msg.data)

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return max(0.0, (self.get_clock().now() - stamp).nanoseconds * 1e-9)

    def _tick(self):
        reason = gate_reason(
            emergency_stop=self.emergency_stop,
            initialized=self.initialized,
            fitness=self.last_fitness,
            fitness_age=self._age(self.last_fitness_time),
            fitness_max=self.fitness_max,
            fitness_timeout=self.fitness_timeout,
            command_age=self._age(self.last_command_time),
            command_timeout=self.command_timeout,
            require_fitness=self.require_fitness,
        )
        is_open = reason == 'open'
        self.cmd_pub.publish(self.last_command if is_open else Twist())

        if reason != self.last_reason:
            log = self.get_logger().info if is_open else self.get_logger().warn
            log(f'cmd_vel gate state: {reason}')
            self.last_reason = reason
        self.open_pub.publish(Bool(data=is_open))
        self.state_pub.publish(String(data=reason))


def main():
    rclpy.init()
    node = ErcCmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish one explicit stop before leaving whenever DDS is still alive.
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
