#include <HardwareSerial.h>

#define fan1_pin 4 // ESP32 pin GPIO16, which connects to the fan the via the relay
#define fan1_pwd 5         // Pin for fan speed control (PWM)
#define fan2_pin 4 // ESP32 pin GPIO16, which connects to the fan the via the relay
#define fan2_pwd 5    

// 1.drain_valve_state 
// 2.water_valve_state 
// 3.nutrients_valve_state 
// 4.plate1_open_valve_state 
// 5.plate1_drain_valve_state 
// 6.plate2_open_valve_state 
// 7.plate2_drain_valve_state 
// 8.pump_state 
// 9.plate1_heater_state 
// 10.plate2_heater_state 
// 11.plate1_fan_state 
// 12.plate2_fan_state 
// 13.led_line1_state 
// 14.led_line2_state 
// 15.led_line3_state 
// 16.mixer

#define actuators [1,3,4,13,12,2,11,18,19,17,16,14,15,2]
HardwareSerial MySerial(1); // UART communication on Serial1

void setup() {
  // Pin setup
  pinMode(fan1_pwd, OUTPUT);
  pinMode(fan1_pin, OUTPUT);
  pinMode(fan2_pwd, OUTPUT);
  pinMode(fan2_pin, OUTPUT);

  // Initialize UART communication
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.begin(115200);
  Serial.println("ESP32 UART Fan Controller Started");

  // Default fan state: stopped
  analogWrite(fan1_pwd , 0);
  analogWrite(fan2_pwd , 0);
}

void loop() {
  // Listen for commands from Raspberry Pi
  Serial.println(MySerial.available());
  if (MySerial.available() > 0) {
    String command = MySerial.readStringUntil('\n');
    command.trim();
    Serial.println("Received command: " + command);

    for(int i=0 ; i<= command.length(); i++){
      char action = command.charAt(index)
      int value = action - '0';
      digitalWrite(actuators[index],value);
    }
    //*****************************************Tank************************************************** */

  //   if (command.startsWith("PLATE 0")){
  //     command.substring(8)
  //   }
  //   //*****************************************Plate 1*********************************************** */
  //   if (command.startsWith("PLATE 1")){
  //     command.substring(8)
  //     if (command.startsWith("FAN_SPEED")) {
  //       digitalWrite(fan1_pin, HIGH); 
  //       int speed = command.substring(10).toInt(); // Extract speed value
  //       speed = constrain(speed, 0, 255);         // Constrain speed to 0-255
  //       analogWrite(fan1_pwd , speed);
  //       Serial.print("Fan speed set to: ");
  //       Serial.println(speed);
  //       MySerial.println("Fan Speed OK");
  //     } 
  //     else if (command == "FAN_STOP") {
  //       analogWrite(fan1_pwd , 0);
  //       digitalWrite(fan1_pin, LOW); 
  //       Serial.println("Fan Stopped");
  //       MySerial.println("Fan Stopped OK");
  //     } 
  //     else {
  //       Serial.println("Unknown command");
  //       MySerial.println("ERROR");
  //     }
  //   }

  //   //*****************************************Plate 2*********************************************** */
  //   if (command.startsWith("PLATE 2")){
  //   command.substring(8)
  //   if (command.startsWith("FAN_SPEED")) {
  //       digitalWrite(fan2_pin, HIGH); 
  //       int speed = command.substring(10).toInt(); // Extract speed value
  //       speed = constrain(speed, 0, 255);         // Constrain speed to 0-255
  //       analogWrite(fan2_pwd , speed);
  //       Serial.print("Fan speed set to: ");
  //       Serial.println(speed);
  //       MySerial.println("Fan Speed OK");
  //     } 
  //     else if (command == "FAN_STOP") {
  //       analogWrite(fan2_pwd , 0);
  //       digitalWrite(fan2_pin, LOW); 
  //       Serial.println("Fan Stopped");
  //       MySerial.println("Fan Stopped OK");
  //     } 
  //     else {
  //       Serial.println("Unknown command");
  //       MySerial.println("ERROR");
  //     }
  //   }
  // }
  delay(1000);
}
}

// /*
//  * This ESP32 code is created by esp32io.com
//  *
//  * This ESP32 code is released in the public domain
//  *
//  * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-controls-pump
//  */

// #define RELAY_PIN 4 // ESP32 pin GPIO16, which connects to the pump the via the relay

// // the setup function runs once when you press reset or power the board
// void setup() {
//   // initialize digital pin GPIO16 as an output.
//   pinMode(RELAY_PIN, OUTPUT);
// }

// // the loop function runs over and over again forever
// void loop() {
//   digitalWrite(RELAY_PIN, HIGH); // turn on +pump 4 seconds
//   delay(10000);
//   digitalWrite(RELAY_PIN, LOW);  // turn off pump 4 seconds
//   delay(10000);
// }
