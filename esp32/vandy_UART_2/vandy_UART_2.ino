void setup() {
  // Start the USB serial monitor
  Serial.begin(115200);

  // Initialize UART2 on pins (RX=16, TX=17)
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 

  Serial.println("UART2 example started");
}

void loop() {
  // If data is available on UART2, read and print it
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("Received on UART2: ");
    Serial.println(msg);
  }

  // If data is available from Serial Monitor, send to UART2
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial2.println(msg);
    Serial.print("Sent to UART2: ");
    Serial.println(msg);
  }
}
