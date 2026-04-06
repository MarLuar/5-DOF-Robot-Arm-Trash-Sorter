// Quick test - upload this to verify sensors are connected
// Based on your working test code

const int bioTrig = 4;
const int bioEcho = 5;
const int nonBioTrig = 2;
const int nonBioEcho = 3;

void setup() {
  Serial.begin(115200);
  pinMode(bioTrig, OUTPUT);
  pinMode(bioEcho, INPUT);
  pinMode(nonBioTrig, OUTPUT);
  pinMode(nonBioEcho, INPUT);
  Serial.println("Sensor test started");
}

void loop() {
  // Test BIO sensor
  digitalWrite(bioTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(bioTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(bioTrig, LOW);
  
  long bioDuration = pulseIn(bioEcho, HIGH);
  float bioDistance = bioDuration * 0.0343 / 2;
  
  delay(50);
  
  // Test NON-BIO sensor
  digitalWrite(nonBioTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(nonBioTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(nonBioTrig, LOW);
  
  long nonBioDuration = pulseIn(nonBioEcho, HIGH);
  float nonBioDistance = nonBioDuration * 0.0343 / 2;
  
  Serial.print("BIO: ");
  Serial.print(bioDistance);
  Serial.print(" cm | NON-BIO: ");
  Serial.print(nonBioDistance);
  Serial.println(" cm");
  
  delay(200);
}
