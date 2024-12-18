#include <HardwareSerial.h>

#define PWM_PIN 5         // Pin for fan speed control (PWM)

HardwareSerial MySerial(1); // UART communication on Serial1

void setup() {
  // Pin setup
  pinMode(PWM_PIN, OUTPUT);

  // Initialize PWM for fan speed
  int freq = 25000;
  int resolution = 8;

  // Initialize UART communication
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.begin(115200);
  Serial.println("ESP32 UART Fan Controller Started");

  // Default fan state: stopped
  analogWrite(PWM_PIN , 0);
}

void loop() {
  // Listen for commands from Raspberry Pi
  Serial.println(MySerial.available());
  if (MySerial.available() > 0) {
    String command = MySerial.readStringUntil('\n');
    command.trim();
    Serial.println("Received command: " + command);
    if (command.startsWith("FAN_SPEED")) {
      int speed = command.substring(10).toInt(); // Extract speed value
      speed = constrain(speed, 0, 255);         // Constrain speed to 0-255
      analogWrite(PWM_PIN , speed);
      Serial.print("Fan speed set to: ");
      Serial.println(speed);
      MySerial.println("Fan Speed OK");
    } else if (command == "FAN_STOP") {
      analogWrite(PWM_PIN , 0);
      Serial.println("Fan Stopped");
      MySerial.println("Fan Stopped OK");
    } else {
      Serial.println("Unknown command");
      MySerial.println("ERROR");
    }
    delay(1000);
  }
}
