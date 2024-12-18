#include <HardwareSerial.h>

#define PWM_PIN 5         // Pin for fan speed control (PWM)
#define DIR_PIN 18        // Pin for fan direction control

HardwareSerial MySerial(1); // UART communication on Serial1

void setup() {
  // Pin setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Initialize PWM for fan speed
  // ledcSetup(0, 25000, 8);  // Channel 0, 25 kHz frequency, 8-bit resolution
  // ledcAttachPin(PWM_PIN, 0); // Attach PWM to PWM_PIN

  // // Initialize UART communication
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.begin(115200);
  Serial.println("ESP32 UART Fan Controller Started");

  // // Default fan state: stopped
  // ledcWrite(0, 0);  // Fan speed 0
  // digitalWrite(DIR_PIN, LOW); // Default direction
}

void loop() {
  // Listen for commands from Raspberry Pi
  if (MySerial.available() > 0) {
    String command = MySerial.readStringUntil('\n');
    command.trim();
    Serial.println("Received command: " + command);
    delay(1000);
    // if (command.startsWith("FAN_SPEED")) {
    //   int speed = command.substring(10).toInt(); // Extract speed value
    //   speed = constrain(speed, 0, 255); // Constrain speed to 0-255
    //   ledcWrite(0, speed);
    //   Serial.print("Fan speed set to: ");
    //   Serial.println(speed);
    //   MySerial.println("Fan Speed OK");
    // }
    // else if (command == "FAN_FORWARD") {
    //   digitalWrite(DIR_PIN, LOW); // Set direction to FORWARD
    //   Serial.println("Fan direction: FORWARD");
    //   MySerial.println("Fan Forward OK");
    // }
    // else if (command == "FAN_REVERSE") {
    //   digitalWrite(DIR_PIN, HIGH); // Set direction to REVERSE
    //   Serial.println("Fan direction: REVERSE");
    //   MySerial.println("Fan Reverse OK");
    // }
    // else if (command == "FAN_STOP") {
    //   ledcWrite(0, 0); // Stop fan (speed = 0)
    //   Serial.println("Fan Stopped");
    //   MySerial.println("Fan Stopped OK");
    // }
    // else {
    //   Serial.println("Unknown command");
    //   MySerial.println("ERROR");
    }
  }
