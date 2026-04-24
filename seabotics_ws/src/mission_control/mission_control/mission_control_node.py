#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from action_msgs.srv import CancelGoal # Tjenesten som stopper Nav2

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        
        # Lytter på systemstatus (fra Mode Manager)
        self.sub_mode = self.create_subscription(Int8, '/system_mode_status', self.mode_callback, 10)
        self.current_mode = 0
        
        # Setter opp en klient for å ringe Nav2 sin globale avbryt-tjeneste
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        self.get_logger().info("Mission Control er online. Venter på Auto-modus...")

    def mode_callback(self, msg):
        prev_mode = self.current_mode
        self.current_mode = msg.data

        # LOGIKK: Sjekk om vi falt UT av Auto-modus (f.eks pga Manuell overstyring eller Killswitch)
        if prev_mode == 2 and self.current_mode != 2:
            self.get_logger().warn("!!! Falt ut av AUTO-modus !!! Avbryter alle Nav2-oppdrag umiddelbart.")
            self.cancel_nav2_goals()

    def cancel_nav2_goals(self):
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Kunne ikke kontakte Nav2 Cancel Service. Sjekk om Nav2 kjører.")
            return
            
        # Sender en tom request. Dette er alt som skal til for at serveren avbryter alle aktive oppdrag.
        req = CancelGoal.Request()
        self.cancel_client.call_async(req)
        self.get_logger().info("Cancel-signal sendt til Nav2. Sikkerhet ivaretatt.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
