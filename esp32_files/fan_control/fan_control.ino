// #include <HardwareSerial.h>

// #define RELAY_PIN 4 // ESP32 pin GPIO16, which connects to the fan the via the relay
// #define PWM_PIN 5         // Pin for fan speed control (PWM)

// HardwareSerial MySerial(1); // UART communication on Serial1

// void setup() {
//   // Pin setup
//   //pinMode(PWM_PIN, OUTPUT);
//   pinMode(RELAY_PIN, OUTPUT);

//   // Initialize UART communication
//   MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
//   Serial.begin(115200);
//   Serial.println("ESP32 UART Fan Controller Started");

//   // Default fan state: stopped
//   //analogWrite(PWM_PIN , 0);
// }

// void loop() {
//   // Listen for commands from Raspberry Pi
//   Serial.println(MySerial.available());
//   if (MySerial.available() > 0) {
//     String command = MySerial.readStringUntil('\n');
//     command.trim();
//     Serial.println("Received command: " + command);
//     if (command.startsWith("FAN_SPEED")) {
//       digitalWrite(RELAY_PIN, HIGH); // turn on fan 10 seconds
//       int speed = command.substring(10).toInt(); // Extract speed value
//       speed = constrain(speed, 0, 255);         // Constrain speed to 0-255
//       //analogWrite(PWM_PIN , speed);
//       Serial.print("Fan speed set to: ");
//       Serial.println(speed);
//       MySerial.println("Fan Speed OK");
//     } else if (command == "FAN_STOP") {
//       //analogWrite(PWM_PIN , 0);
//       digitalWrite(RELAY_PIN, LOW); // turn on fan 10 seconds
//       Serial.println("Fan Stopped");
//       MySerial.println("Fan Stopped OK");
//     } else {
//       Serial.println("Unknown command");
//       MySerial.println("ERROR");
//     }
//   }
//   delay(1000);
// }

/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-controls-pump
 */

#define RELAY_PIN 4 // ESP32 pin GPIO16, which connects to the pump the via the relay

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin GPIO16 as an output.
  pinMode(RELAY_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(RELAY_PIN, HIGH); // turn on +pump 4 seconds
  delay(10000);
  digitalWrite(RELAY_PIN, LOW);  // turn off pump 4 seconds
  delay(10000);
}
