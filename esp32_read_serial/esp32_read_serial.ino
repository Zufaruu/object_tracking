byte b;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void  loop() {
  while (!Serial.available());
  // x = Serial.readString().toInt();

  while (Serial.available() > 0) {
    byte b = Serial.read();
    Serial.print(b);
  }
  
}