#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import asyncio
import threading
import struct
from bleak import BleakClient

# Konstanter for Bluetooth
MAC_ADDRESS = "18:7A:3E:9C:6C:77"
RX_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHANNELS = [
    "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
    "6e400004-b5a3-f393-e0a9-e50e24dcca9e",
    "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
]
# Spør om standard data fra BMS
JBD_COMMAND = bytearray([0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77])

class BMSDriverNode(Node):
    def __init__(self):
        super().__init__('bms_driver_node')
        
        # Setter opp datastrømmen (topic) for rådata
        self.publisher_ = self.create_publisher(BatteryState, '/power/raw_battery_state', 10)
        
        # Variabler som deles mellom ROS2-tråden og Bluetooth-tråden
        self.lock = threading.Lock()
        self.latest_voltage = 0.0
        self.latest_current = 0.0
        self.latest_capacity = 0.0
        self.latest_soc = 0.0
        self.is_connected = False
        
        self.buffer = bytearray()
        
        # ROS2 Timer: Publiserer de nyeste dataene 1 gang i sekundet
        self.timer = self.create_timer(1.0, self.publish_state)
        
        # Starter Bluetooth i en egen bakgrunnstråd slik at det ikke blokkerer ROS2
        self.ble_thread = threading.Thread(target=self.start_ble_loop, daemon=True)
        self.ble_thread.start()

    def publish_state(self):
        """Kjøres av ROS2-løkken. Henter nyeste data trygt via låsen og publiserer."""
        msg = BatteryState()
        
        with self.lock:
            msg.voltage = float(self.latest_voltage)
            msg.current = float(self.latest_current)
            msg.charge = float(self.latest_capacity)
            msg.percentage = float(self.latest_soc / 100.0)
            msg.present = self.is_connected
            
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFEPO4
        self.publisher_.publish(msg)

    def start_ble_loop(self):
        """Dette er hovedfunksjonen for bakgrunnstråden."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ble_task())

    async def ble_task(self):
        """Den asynkrone oppgaven som holder Bluetooth-forbindelsen i live."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Kobler til BMS: {MAC_ADDRESS}")
                async with BleakClient(MAC_ADDRESS, timeout=10.0) as client:
                    
                    with self.lock:
                        self.is_connected = True
                    self.get_logger().info("--- Bluetooth tilkoblet ---")
                    
                    for tx in TX_CHANNELS:
                        try:
                            await client.start_notify(tx, self.notification_handler)
                        except Exception:
                            pass # Hopper over kanaler som ikke støtter varsling
                    
                    # Hovedløkke når tilkoblet
                    while client.is_connected and rclpy.ok():
                        try:
                            await client.write_gatt_char(RX_CHAR, JBD_COMMAND, response=False)
                        except Exception as e:
                            self.get_logger().warn(f"Kunne ikke skrive til batteri: {e}")
                        await asyncio.sleep(1.0) # Spør etter data 1 gang i sekundet
                        
            except Exception as e:
                with self.lock:
                    self.is_connected = False
                self.get_logger().error(f"Bluetooth-feil (Tilkobling mistet): {e}. Prøver igjen om 5 sekunder...")
                await asyncio.sleep(5.0)

    def notification_handler(self, sender, data):
        """Mottar svar fra batteriet."""
        self.buffer.extend(data)
        
        # Sjekker om vi har fått starten på en JBD-pakke
        while len(self.buffer) >= 4:
            if self.buffer[0] != 0xDD:
                self.buffer.pop(0) # Søppeldata, slett første byte og let videre
                continue
                
            if self.buffer[1] == 0x03:
                status = self.buffer[2]
                data_len = self.buffer[3]
                total_len = 4 + data_len + 3 # Header(4) + Data + Sjekksum(2) + Sluttbyte(1)
                
                if len(self.buffer) >= total_len:
                    packet = self.buffer[:total_len]
                    self.buffer = self.buffer[total_len:] # Fjern lest pakke fra bufferet
                    
                    if status == 0x00:
                        self.parse_payload(packet[4:4+data_len])
                else:
                    break # Vent på flere bytes
            else:
                self.buffer.pop(0) # Ukjent pakketype

    def parse_payload(self, data):
        """Dekoder bytes til faktiske tall og oppdaterer delte variabler."""
        if len(data) < 23:
            return
            
        volts_raw = struct.unpack('>H', data[0:2])[0]
        amps_raw = struct.unpack('>h', data[2:4])[0] # Signed (strøm kan være negativ)
        cap_raw = struct.unpack('>H', data[4:6])[0]
        soc_raw = data[19]
        
        # Oppdaterer variablene via låsen så ROS2 ikke leser halvskrevne verdier
        with self.lock:
            self.latest_voltage = volts_raw * 0.01
            self.latest_current = amps_raw * 0.01
            self.latest_capacity = cap_raw * 0.01
            self.latest_soc = soc_raw

def main(args=None):
    rclpy.init(args=args)
    node = BMSDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
