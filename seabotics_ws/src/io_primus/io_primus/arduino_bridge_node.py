#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
import serial
import math

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # --- 1. SENSOR PROTOKOLL KART (Lazy Initialization) ---
        # For å legge til ny sensor: La Arduino printe "Bokstav:Verdi\n" (f.eks "T:12.5")
        self.SENSOR_MAP = {
            'W': '/sensors/water_leak_raw',  # Analog verdi (0-1023)
            'T': '/sensors/temperature',     # Grader Celsius
        }
        self.sensor_publishers = {} # Her lagres publishers dynamisk kun når de trengs

        # --- 2. FYSISKE KONSTANTER (T200 Thrustere) ---
        self.c_fwd = (3.71 * 9.81) / 160000.0
        self.c_rev = (2.92 * 9.81) / 160000.0

        # --- 3. SERIELL TILKOBLING ---
        self.serial_port_name = '/dev/ttyACM0' # Sjekk at denne stemmer med Jetson
        self.baud_rate = 115200
        self.arduino = None
        self.connect_serial()

        # --- 4. SUBSCRIBERS ---
        # Lytter KUN på ferdig allokerte krefter (fra Thrust Allocation)
        self.sub_forces = self.create_subscription(Float64MultiArray, '/thruster_forces', self.force_callback, 10)

        # --- 5. TIMERS ---
        # Lese sensorer fra Arduino (100 Hz for å ikke blokkere buffer)
        self.create_timer(0.01, self.read_serial_data)

        self.get_logger().info("IO Primus: Arduino Bridge er oppe og kjører.")

    def connect_serial(self):
        try:
            self.arduino = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0)
            self.get_logger().info(f"Koblet til Arduino på {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Klarte ikke koble til Arduino: {e}")

    # --- SENSOR MOTTAK (FRA ARDUINO) ---
    def read_serial_data(self):
        if not self.arduino or not self.arduino.is_open:
            return

        try:
            while self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if not line or ":" not in line:
                    continue
                
                # Splitter f.eks "T:12.5" til 'T' og '12.5'
                parts = line.split(":")
                sensor_id = parts[0]
                
                try:
                    sensor_val = float(parts[1])
                except ValueError:
                    continue # Ignorerer korrupt data

                # Lazy Initialization: Opprett topic hvis den ikke finnes
                if sensor_id in self.SENSOR_MAP:
                    topic_name = self.SENSOR_MAP[sensor_id]
                    if sensor_id not in self.sensor_publishers:
                        self.sensor_publishers[sensor_id] = self.create_publisher(Float32, topic_name, 10)
                        self.get_logger().info(f"Oppdaget ny sensor '{sensor_id}'. Opprettet topic: {topic_name}")
                    
                    # Publiser data
                    msg = Float32()
                    msg.data = sensor_val
                    self.sensor_publishers[sensor_id].publish(msg)

        except Exception as e:
            pass # Ignorer lette seriell-feil under lesing

    # --- MOTOR KONTROLL (TIL ARDUINO) ---
    def calculate_pwm(self, force):
        if force >= 0:
            pwm = 1500 + math.sqrt(force / self.c_fwd)
        else:
            pwm = 1500 - math.sqrt(abs(force) / self.c_rev)
        return max(1100, min(1900, int(pwm))) # Safety Clamping

    def force_callback(self, msg):
        if not self.arduino or not self.arduino.is_open:
            return

        forces = msg.data
        if len(forces) != 4:
            return

        pwm1 = self.calculate_pwm(forces[0])
        pwm2 = self.calculate_pwm(forces[1])
        pwm3 = self.calculate_pwm(forces[2])
        pwm4 = self.calculate_pwm(forces[3])

        # Format: <1500,1500,1500,1500>\n
        serial_msg = f"<{pwm1},{pwm2},{pwm3},{pwm4}>\n"
        self.arduino.write(serial_msg.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino and node.arduino.is_open:
            # Send stopp-signal ved avslutning
            node.arduino.write(b"<1500,1500,1500,1500>\n")
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
