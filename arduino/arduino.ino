#define PUMP_PIN 5 // GPIO pin connected to the relay module

void setup() {
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW); // Ensure the pump is off at startup

    Serial1.begin(9600); // Communication with the Raspberry Pi
    Serial.begin(9600);  // For debugging via Serial Monitor
    Serial.println("Arduino Mega ready to receive commands.");
}

void loop() {
    if (Serial1.available() > 0) {
        String command = Serial1.readStringUntil('\n');
        command.trim(); // Remove extra whitespace

        if (command == "on") {
            digitalWrite(PUMP_PIN, HIGH); // Turn the pump on
            Serial.println("Pump turned ON.");
        } else if (command == "off") {
            digitalWrite(PUMP_PIN, LOW); // Turn the pump off
            Serial.println("Pump turned OFF.");
        } else {
            Serial.println("Unknown command received.");
        }
    }
}
