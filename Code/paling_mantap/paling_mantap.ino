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
int step_valve = 0;
double mvalve ;
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
double setPoint;
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
void cacahPulsa()
{
  pulsa_sensor++;
}
char command = 0;

void setup()
{
  Serial.begin(9600);
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
  tinggi1 = constrain(tinggi1,0,48);

  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 * 0.034 / 2 );
  total2 = total2 - readings2[index2];
  readings2[index2] = distance2;
  total2 = total2 + readings2[index2];
  index2 = index2 + 1;
  if (index2 >= 5)
  { // jika sudah mencapai batas pembacaan
    index2 = 0; // kembali ke indeks pertama
  }
  average2 = total2 / 5;
  tinggi2 = 31  - average2;
  tinggi2 = constrain(tinggi2,0,31);
  
  waktuAktual = millis();
  if (waktuAktual >= (waktuLoop + 1000))
  {
    waktuLoop = waktuAktual;
    literPerjam = (pulsa_sensor * 60 / 7.5);
    literPermenit = literPerjam / 60;
    pulsa_sensor = 0;
  }
  literPermenit = constrain(literPermenit,0,17);
  //  SEND SENSOR DATA
  String Data = String(tinggi1) + "," + String (tinggi2) + "," + String(literPermenit) + ","  + String(mvalve);
  Serial2.println(Data);
  delay(100);
  // ON OFF Pump
  if (Serial3.available() > 0)
  {
    command = Serial3.read();
    if (command == '1')
    { //ON
      digitalWrite(relayPin, LOW);
    }
    else if (command == '0')
    { //OFF
      digitalWrite(relayPin, HIGH);
      tangki = 0;
    }
    if (command == '2') {
      tangki = 1;
      Serial.print("Tangki 1");
    }
    else if (command == '3') {
      tangki = 2;
      Serial.print("Tangki 2");
    }
    else if (command == '4')
    { //OFF
      tangki = 3;
      Serial.print("Manual");
    }
  }

  if (Serial2.available() > 0)
  { input = Serial2.readStringUntil('\n');
    // Memanggil fungsi pemisahan nilai
    splitValues(input);
    Serial.println(Kp);
    Serial.println(Ki);
    Serial.println(Kd);
    Serial.println(setPoint);
    Serial.println(mvalve);
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
    float c = outputPID2;         //output PID yg berupa flow ke besar step valve
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
    stepper.moveTo(mvalve);          //Menutup vlave
    stepper.runToPosition();
  }
  else if (tangki == 0) {
    stepper.moveTo(50);          //Menutup vlave
    stepper.runToPosition();
  }
}

void splitValues(String input) {
  input.remove(0, 2);  // Menghilangkan karakter 'P'

  int comma1 = input.indexOf(',');
  int comma2 = input.indexOf(',', comma1 + 1);
  int comma3 = input.indexOf(',', comma2 + 1);
  int comma4 = input.indexOf(',', comma3 + 1);

  // Mengganti koma dengan titik
  input.replace(",", ".");

  Kp = input.substring(0, comma1).toDouble();
  Ki = input.substring(comma1 + 1, comma2).toDouble();
  Kd = input.substring(comma2 + 1, comma3).toDouble();
  setPoint = input.substring(comma3 + 1, comma4).toDouble();
  mvalve = input.substring(comma4 + 1).toDouble();

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
