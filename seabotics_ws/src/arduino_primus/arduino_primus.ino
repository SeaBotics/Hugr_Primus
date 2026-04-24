#include <Servo.h>

// Motorer
Servo m1, m2, m3, m4;

// Sensor Pins
const int waterPin = A0; 

// Watchdog & Timers
unsigned long last_heartbeat = 0;
unsigned long last_water_send = 0;

void setup() {
  Serial.begin(115200);
  
  m1.attach(3);
  m2.attach(5);
  m3.attach(6);
  m4.attach(9);
  
  stopMotors();
}

void loop() {
  // 1. LES SERIELL DATA (Non-blocking format: <1500,1500,1500,1500>)
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    
    if (incoming.startsWith("<") && incoming.endsWith(">")) {
      incoming = incoming.substring(1, incoming.length() - 1); // Fjern < og >
      
      int comma1 = incoming.indexOf(',');
      int comma2 = incoming.indexOf(',', comma1 + 1);
      int comma3 = incoming.indexOf(',', comma2 + 1);
      
      if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
        int val1 = incoming.substring(0, comma1).toInt();
        int val2 = incoming.substring(comma1 + 1, comma2).toInt();
        int val3 = incoming.substring(comma2 + 1, comma3).toInt();
        int val4 = incoming.substring(comma3 + 1).toInt();
        
        // HARDWARE CLAMPING (Dobbel sikkerhet)
        m1.writeMicroseconds(constrain(val1, 1100, 1900));
        m2.writeMicroseconds(constrain(val2, 1100, 1900));
        m3.writeMicroseconds(constrain(val3, 1100, 1900));
        m4.writeMicroseconds(constrain(val4, 1100, 1900));
        
        last_heartbeat = millis(); // Nullstill Watchdog!
      }
    }
  }

  // 2. WATCHDOG (100 ms)
  // Hvis jetson fryser eller kabelen trekkes ut, stopper vi Tvert.
  if (millis() - last_heartbeat > 100) {
    stopMotors();
  }

  // 3. SENSOR DATA RETUR (Hvert 2. sekund for ikke å spamme)
  if (millis() - last_water_send > 2000) {
    int waterVal = analogRead(waterPin);
    Serial.print("W:");
    Serial.println(waterVal);
    last_water_send = millis();
  }
}

void stopMotors() {
  m1.writeMicroseconds(1500);
  m2.writeMicroseconds(1500);
  m3.writeMicroseconds(1500);
  m4.writeMicroseconds(1500);
}