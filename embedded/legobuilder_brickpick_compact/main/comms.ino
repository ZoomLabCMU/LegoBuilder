#include <WiFi.h>
#include "BrickPick.h"

WiFiServer server(80);

void init_server(const char *ssid, const char *password) {
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();
}

String get_ip_string() {
  return WiFi.localIP().toString();
}

int handle_clients(BrickPick &brickpick) {
  // return 0 for client
  // return 1 for no client
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    String request;                         // forward declare a request string to fill when a GET request is seen
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') { // End of line

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request
          // execute the request and block until completed
          // send a response
          if (currentLine.length() == 0) {
            // Set new command and wait for OK or ERROR codes
            brickpick.set_command(request.c_str(), request.length());

            // Needs expansion when position commands are added
            while (!brickpick.request_complete()) {
              delayMicroseconds(10);
            }

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // End of line condition
            if (currentLine.startsWith("GET")) {
              request = prune_get_request(currentLine);
            }
            // Clear current line
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
    return 0;
  }
  return 1;
}

String prune_get_request(String http_get_request) {
  // TODO: this is generally unsafe
  // "GET /{request}?{param_1}={param_1_val} HTTP/1.1"
  String request = http_get_request.substring(4, http_get_request.length() - 9);
  // "/{request}?{param_1}={param_1_val}"
  return request;
}