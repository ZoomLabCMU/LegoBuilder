void wifi_init() {
  Serial.println("Starting ESP01");
  delay(2000);
  Serial1.println("AT");
  Serial.println(Serial1.read());
  Serial1.println("AT+GMR"); //Response: "OK"
  delay(500);
  Serial1.println("AT+CWMODE=0"); //Response: "STA"
  delay(500);
  Serial1.println("AT+CIFSR"); //Response: "AT+CIFSR 192.168.0.105\nOK"
  delay(500);
  Serial1.println("AT+CIPMUX=1"); //Response: "Multiple"
  delay(500);
  Serial1.println("AT+CIPSERVER=1,80"); //Response: "mode 0 to close server mode; mode 1 to open; port = port"
  delay(500);
  printIPAddr();
}

void printIPAddr(){
  while (Serial1.available() > 0){
    char ch = Serial1.read();
    Serial.print(ch);
  }
}


// Untested
int wifi_transmit(char buf[], int n) {
  char AT_cmd[] = {"\0"};
  char msg[n];
  sprintf(AT_cmd, "AT+CIPSEND=%d", n);
  Serial1.println(AT_cmd);
  strncpy(msg, buf, n);
  return 0;
}

// Untested
int wifi_recieve(char buf[], int n) {
  Serial1.readBytesUntil('\n', buf, n);
  return 0;
}

// Untested
