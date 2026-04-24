#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from collections import deque
import time

class BatteryAnalyticsNode(Node):
    def __init__(self):
        super().__init__('battery_analytics_node')
        
        # Abonnent: Lytter til rådataene fra driveren
        self.subscription = self.create_subscription(
            BatteryState,
            '/power/raw_battery_state',
            self.raw_state_callback,
            10
        )
        
        # Utgivere: Sender prosessert data til GUI
        self.state_pub = self.create_publisher(BatteryState, '/power/state', 10)
        self.runtime_pub = self.create_publisher(Float32, '/power/estimated_runtime_min', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Variabler for glidende gjennomsnitt (Smoothing)
        self.current_history = deque(maxlen=10) # Tar vare på de 10 siste målingene
        self.last_msg_time = time.time()
        
        # Watchdog Timer: Sjekker hver 2. sekund om vi har mistet forbindelsen
        self.watchdog_timer = self.create_timer(2.0, self.check_watchdog)

    def raw_state_callback(self, msg: BatteryState):
        self.last_msg_time = time.time()
        
        # 1. Smoothing av strømtrekk
        self.current_history.append(msg.current)
        avg_current = sum(self.current_history) / len(self.current_history)
        
        # Oppretter ny, renset melding
        smooth_msg = msg
        smooth_msg.current = float(avg_current)
        self.state_pub.publish(smooth_msg)
        
        # 2. Beregn gjenværende kjøretid (Runtime)
        runtime_min = -1.0 # Standardverdi for "ukjent" eller "lader"
        if avg_current < -0.5: # Hvis vi trekker mer enn 0.5 Ampere (forbruk er negativt)
            # Tid = Kapasitet (Ah) / Forbruk (A). Ganger med 60 for minutter.
            # msg.charge er antall Ampere-timer igjen på batteriet.
            runtime_hours = msg.charge / abs(avg_current)
            runtime_min = runtime_hours * 60.0
            
        runtime_msg = Float32()
        runtime_msg.data = float(runtime_min)
        self.runtime_pub.publish(runtime_msg)
        
        # 3. Diagnostikk og Alarmer
        self.publish_diagnostics(msg.percentage, msg.present)

    def publish_diagnostics(self, soc_percent, is_present):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "Power: Haicen 75Ah Battery"
        status.hardware_id = "BMS_01"
        
        if not is_present:
            status.level = DiagnosticStatus.ERROR
            status.message = "MISTET BLUETOOTH FORBINDELSE"
        elif soc_percent < 0.10: # Under 10%
            status.level = DiagnosticStatus.ERROR
            status.message = "KRITISK LAVT BATTERI"
        elif soc_percent < 0.20: # Under 20%
            status.level = DiagnosticStatus.WARN
            status.message = "Lavt batteri, vurder retur"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "System OK"
            
        # Legger ved ekstra info som GUI kan plukke opp
        status.values.append(KeyValue(key="SoC", value=f"{soc_percent*100:.1f}%"))
        
        diag_msg.status.append(status)
        self.diag_pub.publish(diag_msg)

    def check_watchdog(self):
        """Sjekker om det er lenge siden vi hørte fra driver-noden."""
        if time.time() - self.last_msg_time > 5.0:
            # Hvis vi ikke har fått data på 5 sekunder, send feilmelding
            self.publish_diagnostics(0.0, False)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryAnalyticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
