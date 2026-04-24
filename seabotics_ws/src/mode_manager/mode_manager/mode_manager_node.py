#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Int8
import time

# TEKNIKER-NOTAT: ANSI Fargekoder for terminal-varsling
RED    = '\033[91m'
YELLOW = '\033[93m'
GREEN  = '\033[92m'
BOLD   = '\033[1m'
ENDC   = '\033[0m'

class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager_node')

        # --- 1. SYSTEM TILSTAND ---
        self.current_mode = 0  # 0=Kill, 1=Manual, 2=Auto
        
        # Watchdog variabler
        self.last_heartbeat = self.get_clock().now()
        self.signal_loss_active = False
        self.watchdog_timeout = 60.0  # Sekunder før Total-Kill
        self.warning_interval = 5.0   # Sekunder mellom hver advarsel
        self.last_warning_time = 0.0

        # --- 2. SUBSCRIBERS (Inndata) ---
        self.sys_cmd_sub = self.create_subscription(Int8, '/system_command', self.sys_cmd_callback, 10)
        # Lytter på Kraft (Newton) fra Xbox:
        self.joy_sub = self.create_subscription(Wrench, '/wrench_joystick', self.joy_callback, 10)
        # Lytter på Fart (m/s) fra Nav2:
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)

        # --- 3. PUBLISHERS (Utdata / Ruting) ---
        # Sender Fart til PID-Regulator (Kun i Auto)
        self.cmd_vel_auto_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        # Sender Kraft direkte til Thrust Allocation (I Manuell og Kill)
        self.thrust_demand_pub = self.create_publisher(Wrench, '/thrust_demand', 10)
        
        # Systemstatus (Lyses opp og Mission Control lytter på denne)
        self.status_pub = self.create_publisher(Int8, '/system_mode_status', 10)

        # --- 4. TIMERE ---
        self.status_timer = self.create_timer(0.1, self.publish_status)   # 10 Hz
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check) # 2 Hz

        self.get_logger().info(f"{BOLD}MODE MANAGER:{ENDC} Aktiv og låst i KILLSWITCH (Modus 0)")

    def update_heartbeat(self):
        if self.signal_loss_active:
            print(f"{GREEN}{BOLD}[SYSTEM] CONNECTION RE-ESTABLISHED!{ENDC}")
            self.signal_loss_active = False
        self.last_heartbeat = self.get_clock().now()

    def sys_cmd_callback(self, msg):
        requested_mode = msg.data
        if requested_mode == 0:
            if self.current_mode != 0:
                self.current_mode = 0
                self.stop_vessel()
                self.get_logger().warn(">>> MODUS ENDRET: 0 (KILLSWITCH) <<<")
        elif requested_mode == 1:
            if self.current_mode != 1:
                self.current_mode = 1
                self.stop_vessel() # Kutt forrige kommando før vi bytter
                self.get_logger().info(">>> MODUS ENDRET: 1 (MANUELL) <<<")
        elif requested_mode == 2:
            if self.current_mode == 1:
                self.current_mode = 2
                self.stop_vessel()
                self.get_logger().info(">>> MODUS ENDRET: 2 (AUTONOM) <<<")
            else:
                self.get_logger().error("AVVIST: Må gå via Manuell før Autonom.")

    # --- Ruting av Manuell Styring (Newton direkte til propellene) ---
    def joy_callback(self, msg):
        self.update_heartbeat() # Xbox holder oss i live
        
        # SIKKERHET: Triumferende sperre!
        if self.current_mode == 0:
            return 
            
        if self.current_mode == 1:
            # Vi er i Manuell. Rute direkte til Thrust Allocation.
            self.thrust_demand_pub.publish(msg)

    # --- Ruting av Auto Styring (Twist til PID-regulator) ---
    def nav_callback(self, msg):
        # SIKKERHET: Triumferende sperre!
        if self.current_mode == 0:
            return

        if self.current_mode == 2:
            self.update_heartbeat() # Nav2 holder oss i live
            # Vi er i Auto. Rute fartsønsket til PID-regulatoren.
            self.cmd_vel_auto_pub.publish(msg)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9

        if self.current_mode != 0:
            if elapsed > 5.0:
                self.signal_loss_active = True
                current_time = time.time()
                if current_time - self.last_warning_time > self.warning_interval:
                    remaining = max(0, int(self.watchdog_timeout - elapsed))
                    print(f"{YELLOW}{BOLD}[!!! WARNING !!!] SIGNAL LOSS DETECTED! {remaining}s UNTIL AUTO-KILL{ENDC}")
                    self.last_warning_time = current_time

            if elapsed > self.watchdog_timeout:
                print(f"{RED}{BOLD}[FATAL] SIGNAL LOST FOR {int(elapsed)}s - EMERGENCY KILLSWITCH!{ENDC}")
                self.current_mode = 0
                self.stop_vessel()

    def stop_vessel(self):
        """Aktiv bremsing: Setter fart og krefter til 0 umiddelbart."""
        zero_twist = Twist()
        zero_wrench = Wrench()
        
        self.cmd_vel_auto_pub.publish(zero_twist)
        self.thrust_demand_pub.publish(zero_wrench)

    def publish_status(self):
        # Kringkast status
        msg = Int8()
        msg.data = self.current_mode
        self.status_pub.publish(msg)
        
        # Hvis vi er i Killswitch, spammer vi STOPP kontinuerlig for dobbel sikkerhet
        if self.current_mode == 0:
            self.stop_vessel()

def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vessel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
