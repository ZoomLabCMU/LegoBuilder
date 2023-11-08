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

  // I use a static client and command flag/variable in order to asynchronously wait for 
  //      blocking commands to complete while leaving a minimal timing footprint so PID & other updates can run
  //      on timers in the main loop. A future iteration might replace this with the AsyncTCP library to do this
  //      in a less messy fashion.
  static WiFiClient client = server.available();   // listen for incoming clients
  static bool command_complete = true;
  static String active_command = "";

  if (!command_complete) { // Check progress on command, send response on termination
    int command_status = brickpick.check_command_status(active_command);
    char response_buf[50];
    switch (command_status) {
      case 0: // command complete
        // Send complete response to client
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        sprintf(response_buf, "Command: {%s} complete!", active_command);
        client.println(response_buf);
        // close the connection:
        client.stop();
        Serial.println("Client Disconnected.");
        // Reset command flag
        command_complete = true;
        active_command = "";
        break;
      case 1: // command pending
        break;
      case 2: // command failed (not completely implemented)
        // Send failed reply to client
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        sprintf(response_buf, "Command: {%s} failed!", active_command);
        client.println(response_buf);
        // close the connection:
        client.stop();
        Serial.println("Client Disconnected.");
        // reset command flag
        command_complete = true;
        active_command = "";
        break;
    }
    return 1; // Exit loop and wait for next call to get new clients
  } else {
    client = server.available();   // listen for incoming clients
  }

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
            // Set new command
            brickpick.set_command(request.c_str(), request.length());
            command_complete = false;
            active_command = request;

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