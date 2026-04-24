#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int8
import os
import sys
import time

# "Dummy" videodriver for å kjøre Pygame uten skjerm
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

class TeleoperationNode(Node):
    def __init__(self):
        super().__init__('teleoperation_node')
        
        # --- 1. KONFIGURASJON ---
        self.DEADZONE    = 0.10
        self.EXPO_POWER  = 1.0

        # --- NYTT: FYSISKE KRAFT-BEGRENSNINGER (Newton) ---
        # Siden 2 thrustere = ca 72.8 N maks, starter vi konservativt på 40 N for testing.
        self.MAX_SURGE_FORCE = 40.0  # Newton fremover/bakover
        self.MAX_SWAY_FORCE  = 40.0  # Newton sidelengs
        self.MAX_YAW_TORQUE  = 20.0  # Newtonmeter (Vridning)

        # --- 2. MAPPING ---
        self.BTN_ARM     = 0            # A-Knapp (Manuell modus)
        self.BTN_REVERS  = 1            # B-Knapp (Reverser Surge)
        self.BTN_INVERT  = 3            # Y-Knapp (Reverser Sway)
        self.BTN_LB      = 4            # Left Bumper (For Killswitch)
        self.BTN_RB      = 5            # Right Bumper (For Killswitch)
        
        self.AXIS_SURGE  = 5            # RT (Gass / Killswitch)
        self.AXIS_SWAY   = 2            # LT (Sway / Killswitch)
        self.AXIS_YAW    = 0            # Venstre Stikke X

        # --- 3. TILSTAND ---
        self.surge_dir = 1
        self.sway_dir  = 1
        self.last_revers_btn = False
        self.last_invert_btn = False
        
        # Tidsmåling og spam-filter
        self.arm_press_start_time = 0.0
        self.is_arming_now = False
        self.is_armed_locally = False
        self.last_published_cmd = -1

        # --- 4. ROS SETUP ---
        # ENDRET: Publiserer nå Wrench på /wrench_joystick
        self.joy_pub = self.create_publisher(Wrench, '/wrench_joystick', 10)
        self.sys_cmd_pub = self.create_publisher(Int8, '/system_command', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz

        # --- 5. PYGAME SETUP ---
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
                self.get_logger().info(f"Pygame fant kontroller: {self.joy.get_name()}")
                self.get_logger().info(f"Maks krefter: Surge={self.MAX_SURGE_FORCE}N, Sway={self.MAX_SWAY_FORCE}N")
            else:
                self.get_logger().error("Ingen kontroller funnet! Sjekk USB.")
                self.joy = None
        except pygame.error as e:
            self.get_logger().error(f"Pygame feil: {e}")
            self.joy = None

    def apply_deadzone_and_curve(self, raw_value, deadzone, expo):
        if abs(raw_value) < deadzone:
            return 0.0
        sign = 1.0 if raw_value > 0 else -1.0
        normalized = (abs(raw_value) - deadzone) / (1.0 - deadzone)
        return sign * (normalized ** expo)

    def timer_callback(self):
        if self.joy is None:
            return
            
        pygame.event.pump()

        # --- SJEKK 1: KILLSWITCH ---
        lt_pressed = self.joy.get_axis(self.AXIS_SWAY) > 0.5
        rt_pressed = self.joy.get_axis(self.AXIS_SURGE) > 0.5
        lb_pressed = self.joy.get_button(self.BTN_LB)
        rb_pressed = self.joy.get_button(self.BTN_RB)

        if lt_pressed and rt_pressed and lb_pressed and rb_pressed:
            if self.last_published_cmd != 0:
                cmd_msg = Int8()
                cmd_msg.data = 0
                self.sys_cmd_pub.publish(cmd_msg)
                self.last_published_cmd = 0
                sys.stdout.write("\r" + " " * 80 + "\r")
                self.get_logger().warn(">>> SYSTEMKOMMANDO: 0 (KILLSWITCH) SENDT <<<")
            self.is_armed_locally = False

        # --- SJEKK 2: MANUELL MODUS ---
        if self.joy.get_button(self.BTN_ARM):
            if not self.is_arming_now:
                self.arm_press_start_time = time.time()
                self.is_arming_now = True
            else:
                if (time.time() - self.arm_press_start_time) >= 2.0:
                    if self.last_published_cmd != 1:
                        cmd_msg = Int8()
                        cmd_msg.data = 1
                        self.sys_cmd_pub.publish(cmd_msg)
                        self.last_published_cmd = 1
                        sys.stdout.write("\r" + " " * 80 + "\r")
                        self.get_logger().info(">>> SYSTEMKOMMANDO: 1 (MANUELL) SENDT <<<")
                    self.is_armed_locally = True
        else:
            self.is_arming_now = False

        # --- SJEKK 3: REVERS & INVERT ---
        current_revers_btn = self.joy.get_button(self.BTN_REVERS)
        if current_revers_btn and not self.last_revers_btn:
            if self.joy.get_axis(self.AXIS_SURGE) < -0.9:
                self.surge_dir *= -1
        self.last_revers_btn = current_revers_btn

        current_invert_btn = self.joy.get_button(self.BTN_INVERT)
        if current_invert_btn and not self.last_invert_btn:
            if self.joy.get_axis(self.AXIS_SWAY) < -0.9:
                self.sway_dir *= -1
        self.last_invert_btn = current_invert_btn

        # --- SJEKK 4: STYRESIGNALER (Nå som WRENCH i Newton) ---
        raw_surge = self.joy.get_axis(self.AXIS_SURGE)
        raw_sway  = self.joy.get_axis(self.AXIS_SWAY)
        raw_yaw   = self.joy.get_axis(self.AXIS_YAW)

        raw_surge = (raw_surge + 1.0) / 2.0
        raw_sway = (raw_sway + 1.0) / 2.0

        # Multipliserer -1.0 til 1.0 verdien med Maks Newton
        val_surge = self.apply_deadzone_and_curve(raw_surge, self.DEADZONE, self.EXPO_POWER) * self.surge_dir * self.MAX_SURGE_FORCE
        val_sway  = self.apply_deadzone_and_curve(raw_sway, self.DEADZONE, self.EXPO_POWER) * self.sway_dir * self.MAX_SWAY_FORCE
        val_yaw   = self.apply_deadzone_and_curve(raw_yaw, self.DEADZONE, self.EXPO_POWER) * self.MAX_YAW_TORQUE

        # ENDRET: Oppretter Wrench-meldingen
        wrench_msg = Wrench()
        wrench_msg.force.x  = float(val_surge)
        wrench_msg.force.y  = float(val_sway)
        wrench_msg.torque.z = float(val_yaw) * -1.0
        
        self.joy_pub.publish(wrench_msg)

        # --- LIVE TERMINAL DASHBOARD ---
        status_txt = "!!! MANUELL !!!" if self.is_armed_locally else "  KILLSWITCH   "
        dir_surge_txt = "FWD" if self.surge_dir == 1 else "REV"
        dir_sway_txt = "STBD" if self.sway_dir == 1 else "PORT"
        
        output_str = f"\r[{status_txt}]  SURGE: {val_surge:+.1f}N  |  SWAY: {val_sway:+.1f}N  |  YAW: {val_yaw:+.1f}Nm      "
        sys.stdout.write(output_str)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\nAvslutter...\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
