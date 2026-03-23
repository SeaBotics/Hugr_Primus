#!/usr/bin/env python3

import math
# Gir tilgang til trigonometriske funksjoner som atan2, sin og cos.
# Disse brukes for å regne ut retning/vinkel til et målpunkt.

import rclpy
# ROS 2-biblioteket for Python.
# Må brukes for å lage noder, publishers, subscribers og kjøre ROS-kommunikasjon.

from rclpy.node import Node
# Importerer Node-klassen.
# Klassen vår arver fra denne for å bli en ROS2-node.

from geometry_msgs.msg import PointStamped
# Meldingstype for et punkt med:
# - header (bl.a. frame_id og timestamp)
# - point.x, point.y, point.z
# Brukes her for målpunkt og posisjon.

from std_msgs.msg import Float64
# Enkel meldingstype med ett tallfelt: data.
# Brukes for yaw og ønsket heading.


class LOSGuidanceNode(Node):
    def __init__(self):
        # Oppretter selve ROS2-noden og gir den navnet "los_guidance_node"
        super().__init__('los_guidance_node')

        # Parametere gjør at topic-navn og grenser kan endres uten å skrive om koden.
        # Dette er nyttig når noden senere skal brukes med launch-filer eller YAML-parametere.
        self.declare_parameter('input_topic', '/guidance/active_midpoint')
        self.declare_parameter('output_topic', '/guidance/desired_heading')
        self.declare_parameter('position_topic', '/state/position')
        self.declare_parameter('yaw_topic', '/state/yaw')
        self.declare_parameter('min_forward_x', 0.05)

        # Leser parameterverdiene ut av ROS2-parameterserveren.
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        position_topic = self.get_parameter('position_topic').value
        yaw_topic = self.get_parameter('yaw_topic').value
        self.min_forward_x = float(self.get_parameter('min_forward_x').value)

        # Intern tilstand:
        # current_position lagrer siste kjente posisjon i map-frame.
        # current_yaw lagrer siste kjente yaw i radianer.
        # Begge starter som None fordi noden ikke har mottatt data ennå.
        self.current_position = None   # (x, y)
        self.current_yaw = None        # yaw [rad]

        # Publisher:
        # Denne sender ut ønsket heading som en Float64-melding.
        # Andre noder, f.eks. heading-kontroller, kan abonnere på denne.
        self.heading_pub = self.create_publisher(Float64, output_topic, 10)

        # Subscriber for målpunktet.
        # Når et nytt målpunkt kommer inn, kjører ROS funksjonen target_callback().
        self.target_sub = self.create_subscription(
            PointStamped,
            input_topic,
            self.target_callback,
            10
        )

        # Subscriber for båtens nåværende posisjon.
        # Dette trengs når målpunktet er gitt i globalt frame ("map").
        self.position_sub = self.create_subscription(
            PointStamped,
            position_topic,
            self.position_callback,
            10
        )

        # Subscriber for båtens nåværende yaw.
        # Også nødvendig når målpunktet er i globalt frame.
        self.yaw_sub = self.create_subscription(
            Float64,
            yaw_topic,
            self.yaw_callback,
            10
        )

        # Oppstartslogger.
        # Disse er kun til debugging og oversikt.
        self.get_logger().info('LOS guidance node started.')
        self.get_logger().info(f'Subscribing target: {input_topic}')
        self.get_logger().info(f'Subscribing position: {position_topic}')
        self.get_logger().info(f'Subscribing yaw: {yaw_topic}')
        self.get_logger().info(f'Publishing desired heading to: {output_topic}')

    def position_callback(self, msg: PointStamped):
        # Hver gang en ny posisjon kommer inn, lagres den.
        # Vi bruker bare x og y her, ikke z.
        self.current_position = (msg.point.x, msg.point.y)

    def yaw_callback(self, msg: Float64):
        # Hver gang ny yaw kommer inn, lagres den.
        # Verdien antas å være i radianer.
        self.current_yaw = msg.data

    def wrap_to_pi(self, angle: float) -> float:
        # Sørger for at en vinkel alltid ligger i intervallet [-pi, pi].
        # Dette er viktig fordi vinkler kan "gå rundt" og ellers bli større enn pi eller mindre enn -pi.
        # Eksempel: 3.5 rad og -2.78 rad kan representere nesten samme retning.
        return math.atan2(math.sin(angle), math.cos(angle))

    def target_callback(self, msg: PointStamped):
        # Dette er hovedfunksjonen i noden.
        # Den kjøres hver gang et nytt målpunkt mottas.

        # frame_id forteller hvilket koordinatsystem målpunktet er gitt i.
        # strip() fjerner eventuelle mellomrom før/etter teksten.
        frame_id = msg.header.frame_id.strip()

        # Leser ut målpunktets koordinater.
        target_x = msg.point.x
        target_y = msg.point.y

        if frame_id == 'base_link':
            # base_link betyr at målpunktet allerede er gitt relativt til båten.
            # Da kan vi tolke:
            # - x som framover/bakover
            # - y som sideveis
            #
            # Dette er den enkleste varianten og passer fint med LOS v1.

            # Hvis målpunktet ligger nesten i origo, hopper vi over beregning.
            # Da er målet så nært båten at headingen blir lite nyttig eller støyete.
            if abs(target_x) < self.min_forward_x and abs(target_y) < self.min_forward_x:
                self.get_logger().warn('Target point too close to origin; skipping heading update')
                return

            # atan2(y, x) gir vinkelen fra båtens foroverretning til målpunktet.
            # Eksempel:
            # - (5, 0)  -> 0 rad
            # - (5, 5)  -> ca +0.785 rad
            # - (5,-5)  -> ca -0.785 rad
            desired_heading = math.atan2(target_y, target_x)

        elif frame_id == 'map':
            # map betyr at målpunktet er gitt i globale koordinater.
            # Da holder det ikke å bruke atan2(target_y, target_x) direkte,
            # fordi target_x og target_y ikke lenger er relativt til båten.

            # Uten posisjon og yaw vet vi ikke hvor båten er eller hvilken vei den peker.
            # Da kan vi ikke regne relativ heading korrekt.
            if self.current_position is None or self.current_yaw is None:
                self.get_logger().warn('Missing position/yaw for map-frame target; skipping heading update')
                return

            # Henter ut båtens nåværende posisjon i map-frame.
            px, py = self.current_position

            # Lager vektoren fra båt -> mål.
            # Dette sier "hvor ligger målet i forhold til båten" i globale koordinater.
            dx = target_x - px
            dy = target_y - py

            # Beregner absolutt peiling/bearing til målet i map-frame.
            # Dette er den globale retningen fra båtens posisjon til target.
            target_bearing = math.atan2(dy, dx)

            # Trekker fra båtens nåværende yaw for å få en relativ headingfeil.
            # wrap_to_pi() sørger for at svaret holder seg i [-pi, pi].
            desired_heading = self.wrap_to_pi(target_bearing - self.current_yaw)

        else:
            # Hvis frame_id er noe annet enn "base_link" eller "map",
            # vet ikke noden hvordan den skal tolke målpunktet.
            self.get_logger().warn(
                f'Unsupported frame_id "{frame_id}". Expected "base_link" or "map".'
            )
            return

        # Lager meldingen som skal publiseres ut.
        heading_msg = Float64()
        heading_msg.data = desired_heading

        # Publiserer ønsket heading.
        self.heading_pub.publish(heading_msg)

        # Logger hva som kom inn og hva som ble regnet ut.
        self.get_logger().info(
            f'frame={frame_id}, target=({target_x:.3f}, {target_y:.3f}) '
            f'-> desired_heading={desired_heading:.3f} rad'
        )


def main(args=None):
    # Starter ROS2-kommunikasjonen.
    rclpy.init(args=args)

    # Oppretter en instans av noden.
    node = LOSGuidanceNode()

    try:
        # Holder noden i live og lar ROS2 håndtere callbacks.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Lar programmet stoppe pent med Ctrl+C.
        pass
    finally:
        # Rydder opp node og ROS2 ved avslutning.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Kjør main() når fila startes direkte.
    main()