double set = 0;
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  // PUMP
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command == "1") {
      //       ON
      Serial3.println('1');
    }
    else if (command == "0") {
      //       OFF
      Serial3.println('0');
    }
    else if (command == "2") {
      //         Tangki 1
      Serial3.println('2');
    }
    else if (command == "3") {
      //      Tangki 2
      Serial3.println('3');
    }
    else if (command == "M") {
      //      Manual
      Serial3.println('4');
    }
    else if (command.startsWith("P")) {
      //      Setpoint
      Serial2.println(command);

    }
  }

  // SENSOR
  if (Serial2.available() > 0) {
    String data = Serial2.readStringUntil('\n');
    Serial.println(data);
  }
}
