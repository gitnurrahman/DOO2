#include <AccelStepper.h>

const int dirPin = 6;
const int stepPin = 8;
const int triggerPin = 12;
const int echoPin = 13;
const int triggerPin2 = 9;
const int echoPin2 = 10;
const int relayPin = 4;
unsigned char pinFlowsensor = 2;

AccelStepper stepper(1, stepPin, dirPin);
int step_valve;
int step_valve1;
const int numReadings = 5;
int readings[numReadings];
int index = 0;
double total = 0;
double average = 0;
double duration, distance;
double tinggi1 = 0;

int readings2[numReadings];
int index2 = 0;
double total2 = 0;
double average2 = 0;
double duration2, distance2;
double tinggi2 = 0;

volatile int pulsa_sensor;
float literPerjam, literPermenit;
unsigned long waktuAktual;
unsigned long waktuLoop;
double liter;

double dt;
double integral, error_sebelumnya, output_PID = 0;
double last_time = 0;
double setPoint ;
double Kp ;
double Ki ;
double Kd ;
unsigned long Millis_sebelumnya = 0;
double errorSum1 = 0.00;
double lastError1 = 0.00;
double outputPID1 = 0.00;        //output PID tank 1
double errorSum2 = 0.00;
double lastError2 = 0.00;
double outputPID2 = 0.00;        //output PID tank 1
int tangki = 0;
String input;
String separator = ",";
void cacahPulsa()
{
  pulsa_sensor++;
}


void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  for (int i = 0; i < numReadings; i++)
  {
    readings[i] = 0;
    readings2[i] = 0;
  }

  pinMode(pinFlowsensor, INPUT);
  digitalWrite(pinFlowsensor, HIGH);
  attachInterrupt(0, cacahPulsa, RISING);
  sei();

  waktuAktual = millis();
  waktuLoop = waktuAktual;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  step_valve = 0;
  step_valve1 = 0;
}

void loop()
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034 / 2 - 0.4393) / 0.9536;
  total = total - readings[index];
  readings[index] = distance;
  total = total + readings[index];
  index = index + 1;
  if (index >= 5)
  {
    index = 0;
  }
  average = total / 5;
  tinggi1 = 48 - average;
  constrain(tinggi1, 0, 48);

  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 * 0.034 / 2 - 0.4393) / 0.9536;
  total2 = total2 - readings2[index2];
  readings2[index2] = distance2;
  total2 = total2 + readings2[index2];
  index2 = index2 + 1;
  if (index2 >= 5)
  { // jika sudah mencapai batas pembacaan
    index2 = 0; // kembali ke indeks pertama
  }
  average2 = total2 / 5;
  tinggi2 = 33 - average2;
  constrain(tinggi2, 0, 33);

  waktuAktual = millis();
  if (waktuAktual >= (waktuLoop + 1000))
  {
    waktuLoop = waktuAktual;
    literPerjam = (pulsa_sensor * 60 / 7.5);
    literPermenit = literPerjam / 60;
    pulsa_sensor = 0;
  }

  //  SEND SENSOR DATA
  String Data = String(tinggi1) + "," + String (tinggi2) + "," + String(literPermenit) + ","  + String(step_valve);
  Serial2.println(Data);
  delay(100);
  // ON OFF Pump
  if (Serial3.available() > 0)
  {
    char command = Serial3.read();
    if (command == '1')
    { //ON
      digitalWrite(relayPin, LOW);
    }
    else if (command == '0')
    { //OFF
      digitalWrite(relayPin, HIGH);
      tangki = 0;
    }
    else if (command == '4')
    { //OFF
      tangki = 3;
    }
  }

  if (Serial1.available() > 0)
  {
    char command = Serial1.read();
    if (command == '2') {
      tangki = 1;
    }
    else if (command == '3') {
      tangki = 2;
    }
  }

  if (Serial2.available() > 0)
  { input = Serial2.readStringUntil('\n');
    // Memanggil fungsi pemisahan nilai
    splitValues(input);
    Serial.print(Kp);
    Serial.print(Ki);
    Serial.print(Kd);
    Serial.print(setPoint);
    Serial.print(step_valve1);
  }
  if (tangki == 1) {
    computePID(setPoint, tinggi1, errorSum1, lastError1, outputPID1);
    float x = outputPID1;         //output PID yg berupa flow ke besar step valve
    float y = (-70) * x + 5400;
    step_valve = y;
    if (step_valve > 5300) {
      step_valve = 5300;
    }
    else {
      step_valve = step_valve;
    }

    stepper.moveTo(step_valve);   //PID gerakkan vlave
    stepper.runToPosition();
  }
  else if (tangki == 2) {
    computePID(setPoint, tinggi2, errorSum2, lastError2, outputPID2);
    float c = outputPID1;         //output PID yg berupa flow ke besar step valve
    float d = (-70) * c + 5400;
    step_valve = d;
    if (step_valve > 5300) {
      step_valve = 5300;
    }
    else {
      step_valve = step_valve;
    }

    stepper.moveTo(step_valve);   //PID gerakkan vlave
    stepper.runToPosition();
  }
  else if (tangki == 3) {
    stepper.moveTo(step_valve1);          //Menutup vlave
    stepper.runToPosition();
  }
  else if (tangki == 0) {
    stepper.moveTo(100);          //Menutup vlave
    stepper.runToPosition();
  }
}
void splitValues(String input) {
  // Mencari indeks pemisah untuk setiap nilai
  int separatorIndex1 = input.indexOf(separator);
  int separatorIndex2 = input.indexOf(separator, separatorIndex1 + 1);
  int separatorIndex3 = input.indexOf(separator, separatorIndex2 + 1);
  int separatorIndex4 = input.indexOf(separator, separatorIndex3 + 1);

  // Memisahkan nilai menggunakan substring
  String kpString = input.substring(1, separatorIndex1);
  String kiString = input.substring(separatorIndex1 + 1, separatorIndex2);
  String kdString = input.substring(separatorIndex2 + 1, separatorIndex3);
  String setPointString = input.substring(separatorIndex3 + 1, separatorIndex4);
  String valveString = input.substring(separatorIndex4 + 1);

  // Mengkonversi nilai ke tipe double dan int
  Kp = kpString.toDouble();
  Ki = kiString.toDouble();
  Kd = kdString.toDouble();
  setPoint = setPointString.toDouble();
  step_valve1 = valveString.toInt();
}
void computePID(double setPoint, double tinggi, double& errorSum, double& lastError, double& outputPID) {
  // Compute time difference
  unsigned long Millis_sekarang = millis();
  double deltaTime = (Millis_sekarang - Millis_sebelumnya) / 1000.00;

  // Compute error and PID output
  double error = setPoint - tinggi;
  errorSum += error * deltaTime;
  double dError = (error - lastError) / deltaTime;
  outputPID = (Kp * error) + (Ki * errorSum) + (Kd * dError);
  if (outputPID > 15) {
    outputPID = 15;
  }
  else if (outputPID < -15) {
    outputPID = -15;
  }
  else {
    outputPID = outputPID;
  }

  Millis_sebelumnya = Millis_sekarang;
  lastError = error;
}
