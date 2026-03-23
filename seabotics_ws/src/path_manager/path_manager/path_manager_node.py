#!/usr/bin/env python3

import math
# Brukes til avstandsberegning med math.hypot().

from typing import List, Optional, Tuple
# Type hints:
# - List[...] for lister
# - Optional[...] for verdier som kan være None
# - Tuple[...] for koordinatpar som (x, y)

import rclpy
# ROS 2-biblioteket for Python.

from rclpy.node import Node
# Importerer Node-klassen som denne ROS2-noden arver fra.

from geometry_msgs.msg import PointStamped, PoseArray
# PointStamped brukes for enkeltpunkter med frame_id og timestamp.
# PoseArray brukes her som en enkel måte å sende inn en liste med gate-midtpunkter.


class PathManagerNode(Node):
    def __init__(self):
        # Oppretter selve ROS2-noden og gir den navnet "path_manager_node".
        super().__init__('path_manager_node')

        # Parametere for topic-navn.
        # Disse gjør det enkelt å endre oppsett uten å skrive om koden.
        self.declare_parameter('gps_target_topic', '/path_manager/gps_target')
        self.declare_parameter('gate_midpoints_topic', '/perception/gate_midpoints')
        self.declare_parameter('position_topic', '/state/position')
        self.declare_parameter('active_target_topic', '/guidance/active_midpoint')

        # Parametere for enkel logikk i path manager.
        self.declare_parameter('publish_period', 0.2)
        # Hvor ofte timer_callback skal kjøre [sek].

        self.declare_parameter('min_forward_x', 0.5)
        # Mindste x-verdi for at en gate skal regnes som "foran båten".

        self.declare_parameter('vision_hold_time', 1.0)
        # Hvor lenge vi holder på siste vision-target etter at deteksjonen forsvinner.

        self.declare_parameter('gps_reached_radius', 2.0)
        # Radius for å regne GPS-target som nådd.
        # Ikke brukt aktivt i timer_callback ennå, men nyttig for senere utvidelse.

        # Leser topic-parametere.
        gps_target_topic = self.get_parameter('gps_target_topic').value
        gate_midpoints_topic = self.get_parameter('gate_midpoints_topic').value
        position_topic = self.get_parameter('position_topic').value
        active_target_topic = self.get_parameter('active_target_topic').value

        # Leser øvrige parametere og lagrer som tall.
        publish_period = float(self.get_parameter('publish_period').value)
        self.min_forward_x = float(self.get_parameter('min_forward_x').value)
        self.vision_hold_time = float(self.get_parameter('vision_hold_time').value)
        self.gps_reached_radius = float(self.get_parameter('gps_reached_radius').value)

        # Intern state:
        # gps_target er fallback-målet fra GPS, i globale koordinater.
        self.gps_target: Optional[Tuple[float, float]] = None

        # current_position er båtens siste kjente posisjon i globale koordinater.
        self.current_position: Optional[Tuple[float, float]] = None

        # detected_gate_targets_body er en liste med gate-midtpunkter sett av kamera.
        # Disse antas å være i body frame:
        # x = framover, y = sideveis relativt til båten.
        self.detected_gate_targets_body: List[Tuple[float, float]] = []

        # current_mode sier hvilken logikk som styrer akkurat nå:
        # 'gps' eller 'vision'
        self.current_mode = 'gps'

        # current_target er det aktive målet som sist ble valgt.
        self.current_target: Optional[Tuple[float, float]] = None

        # last_vision_time brukes for å vite når vi sist hadde en gyldig vision-detektering.
        self.last_vision_time: Optional[float] = None

        # Subscriber for GPS-målet.
        # Dette er et fallback-target som brukes når vision ikke har noe bedre mål.
        self.gps_sub = self.create_subscription(
            PointStamped,
            gps_target_topic,
            self.gps_target_callback,
            10
        )

        # Subscriber for gate-midtpunkter fra perception.
        # Disse kommer som PoseArray, hvor vi bare bruker pose.position.x/y.
        self.gates_sub = self.create_subscription(
            PoseArray,
            gate_midpoints_topic,
            self.gate_midpoints_callback,
            10
        )

        # Subscriber for båtens nåværende posisjon.
        # Brukes bl.a. for å kunne sjekke avstand til GPS-mål.
        self.position_sub = self.create_subscription(
            PointStamped,
            position_topic,
            self.position_callback,
            10
        )

        # Publisher for aktivt mål som skal sendes videre til LOS-guidance.
        self.active_target_pub = self.create_publisher(
            PointStamped,
            active_target_topic,
            10
        )

        # Timer som jevnlig kjører beslutningslogikken i path manager.
        self.timer = self.create_timer(publish_period, self.timer_callback)

        # Oppstartslogger for oversikt og debugging.
        self.get_logger().info('Path manager node started.')
        self.get_logger().info(f'GPS target topic: {gps_target_topic}')
        self.get_logger().info(f'Gate midpoints topic: {gate_midpoints_topic}')
        self.get_logger().info(f'Position topic: {position_topic}')
        self.get_logger().info(f'Active target topic: {active_target_topic}')

    def gps_target_callback(self, msg: PointStamped):
        # Lagrer siste mottatte GPS-target som et koordinatpar (x, y).
        self.gps_target = (msg.point.x, msg.point.y)

    def gate_midpoints_callback(self, msg: PoseArray):
        # Leser alle gate-midtpunkter fra PoseArray og lagrer dem som en liste av (x, y).
        self.detected_gate_targets_body = [
            (pose.position.x, pose.position.y)
            for pose in msg.poses
        ]

        # Hvis vi faktisk har mottatt minst ett gate-target, lagres tidspunktet.
        # Dette brukes senere for "vision_hold_time".
        if len(self.detected_gate_targets_body) > 0:
            self.last_vision_time = self.get_clock().now().nanoseconds * 1e-9

    def position_callback(self, msg: PointStamped):
        # Lagrer siste kjente båtposisjon.
        self.current_position = (msg.point.x, msg.point.y)

    def filter_valid_gates(self, gates_body: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        # Filtrerer bort alle gate-targets som ikke ligger foran båten.
        # Kun targets med x > min_forward_x beholdes.
        return [(x, y) for (x, y) in gates_body if x > self.min_forward_x]

    def choose_best_gate(self, gates_body: List[Tuple[float, float]]) -> Tuple[float, float]:
        # Velger "beste" gate blant de gyldige.
        # Her brukes enkleste regel: nærmeste gate foran båten.
        return min(gates_body, key=lambda p: math.hypot(p[0], p[1]))

    def gps_target_reached(self) -> bool:
        # Sjekker om GPS-target er nådd.
        # Returnerer False hvis vi mangler enten GPS-target eller båtposisjon.
        if self.gps_target is None or self.current_position is None:
            return False

        gx, gy = self.gps_target
        px, py = self.current_position

        # Avstanden mellom båt og GPS-target sammenlignes med gps_reached_radius.
        return math.hypot(gx - px, gy - py) < self.gps_reached_radius

    def vision_recently_available(self) -> bool:
        # Returnerer True hvis vi nylig har hatt en vision-detektering.
        # Dette brukes for å unngå at systemet hopper for raskt tilbake til GPS
        # når kamera mister target i et kort øyeblikk.
        if self.last_vision_time is None:
            return False

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return (now_sec - self.last_vision_time) < self.vision_hold_time

    def publish_active_target(self, x: float, y: float, frame_id: str):
        # Lager og publiserer et PointStamped-mål.
        # frame_id angir hvilket koordinatsystem targetet ligger i.
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.active_target_pub.publish(msg)

    def timer_callback(self):
        # Hovedlogikken i path manager.
        # Denne kjører periodisk og bestemmer hvilket mål som skal være aktivt.

        # Først filtreres gate-targets slik at bare mål foran båten beholdes.
        valid_gates = self.filter_valid_gates(self.detected_gate_targets_body)

        # Prioritet 1: vision-targets hvis vi har gyldige gates.
        if len(valid_gates) > 0:
            best_gate = self.choose_best_gate(valid_gates)
            self.current_mode = 'vision'
            self.current_target = best_gate

            # Publiserer i body frame fordi LOS v1 forventer relative mål.
            self.publish_active_target(best_gate[0], best_gate[1], 'base_link')

            self.get_logger().debug(
                f'Mode=vision target=({best_gate[0]:.2f}, {best_gate[1]:.2f})'
            )
            return

        # Hvis vision nettopp forsvant, kan vi holde på siste vision-target en kort stund.
        # Dette gir litt hysterese og gjør systemet mindre hakkete.
        if self.current_mode == 'vision' and self.vision_recently_available() and self.current_target is not None:
            tx, ty = self.current_target
            self.publish_active_target(tx, ty, 'base_link')
            return

        # Prioritet 2: GPS fallback hvis vi ikke har et brukbart vision-target.
        if self.gps_target is not None:
            self.current_mode = 'gps'
            self.current_target = self.gps_target

            # GPS-target publiseres i map frame fordi det er globalt.
            self.publish_active_target(self.gps_target[0], self.gps_target[1], 'map')

            self.get_logger().debug(
                f'Mode=gps target=({self.gps_target[0]:.2f}, {self.gps_target[1]:.2f})'
            )
            return

        # Hvis verken vision eller GPS er tilgjengelig, finnes det ikke noe aktivt mål.
        self.current_target = None


def main(args=None):
    # Standard oppstart for ROS2 Python-node.
    rclpy.init(args=args)
    node = PathManagerNode()

    try:
        # Holder noden kjørende og lar ROS2 håndtere callbacks.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Lar programmet stoppe pent med Ctrl+C.
        pass
    finally:
        # Rydder opp node og ROS2-kommunikasjon ved avslutning.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Kjør main() når fila startes direkte.
    main()