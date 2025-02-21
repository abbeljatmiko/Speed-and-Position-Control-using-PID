  #include <driver/timer.h>
  #include <WiFi.h>
  #include <PID_v1_bc.h>

  hw_timer_t *timer = NULL;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  volatile bool timerFlag = false;  

  const char* ssid = "lebba";
  const char* password = "abbelj2828";
  WiFiServer server(80);
  String header;

  #define PPR 400
  const int RPWM_PIN = 25;
  const int LPWM_PIN = 26;
  const int PIN_A = 32;
  const int PIN_B = 33;
  int PWM = 0;
  int constrainedValue;

  float Kp = 0.2;
  float Ki = 0.1;
  float Kd = 0.05;
  float integral = 0;
  float derivative;
  float error;
  float errorSebelumnya = 0;
  float outputPID;

  float posisi_deg;
  volatile long delta_posisi;
  float speed_setpoint = 2000;
  float posisiSebelumnya;
  volatile long int posisi;
  float kecepatan;
  volatile int flag;

  void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    timerFlag = true;
    delta_posisi = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
  }

  void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();

    pinMode(RPWM_PIN, OUTPUT);
    pinMode(LPWM_PIN, OUTPUT);
    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_A), ISR_INT0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), ISR_INT1, CHANGE);

    // Konfigurasi Timer untuk versi terbaru ESP32
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1us per tick), count up
    timerAttachInterrupt(timer, &onTimer, true); // Attach interrupt
    timerAlarmWrite(timer, 1000000, true); // 1 detik (1.000.000 us)
    timerAlarmEnable(timer); // Enable timer alarm
  }

  void ISR_INT0() {
    int pinA = digitalRead(PIN_A);
    int pinB = digitalRead(PIN_B);

    if (pinA == LOW && pinB == LOW) {
      posisi--; // CCW
    } else if (pinA == LOW && pinB == HIGH) {
      posisi++;
    } else if (pinA == HIGH && pinB == LOW) {
      posisi++; // CW
    } else if (pinA == HIGH && pinB == HIGH) {
      posisi--; // CCW
    }
  }

  void ISR_INT1() {
    int pinA = digitalRead(PIN_A);
    int pinB = digitalRead(PIN_B);

    if (pinA == LOW && pinB == LOW) {
      posisi++; // CW
    } else if (pinA == LOW && pinB == HIGH) {
      posisi--;
    } else if (pinA == HIGH && pinB == LOW) {
      posisi--; // CCW
    } else if (pinA == HIGH && pinB == HIGH) {
      posisi++; // CW
    }
  }

  void loop() {
    WiFiClient client = server.available();
    if (client) {
      String currentLine = "";
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          header += c;
          if (c == '\n') {
            if (currentLine.length() == 0) {
              processHTTPRequest(client);
              break;
            } else {
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }
        }
      }
      header = "";
      client.stop();
    }
    if(timerFlag){
      portENTER_CRITICAL(&timerMux);
      timerFlag = false;
      delta_posisi = 0;
      portEXIT_CRITICAL(&timerMux);

      posisi_deg = ((posisi / 4) * 360 / PPR) % 360;
      delta_posisi = ((posisi / 4) * 360 / PPR);
      kecepatan = delta_posisi - posisiSebelumnya; //

      error = speed_setpoint - kecepatan;
      integral += error;
      integral = constrain(integral, -1000, 1000);
      derivative = error - errorSebelumnya;
      errorSebelumnya = error;
    
      outputPID = Kp * error + Ki * integral + Kd * derivative;
      constrainedValue = constrain(abs(outputPID), 0, 100);
      //PWM = map(constrainedValue, 0, 100, 0, 255);
      int pwm_change = Kp*error;
      PWM = constrain(PWM + (pwm_change > 0 ? 1 : -1), 0, 255);
      analogWrite(RPWM_PIN, outputPID >= 0 ? PWM : 0);
      analogWrite(LPWM_PIN, outputPID < 0 ? PWM : 0);

      Serial.print("outputPID: "); Serial.print(outputPID);
      Serial.print(" PWM: "); Serial.print(PWM);
      Serial.print("   Kecepatan (deg/s): "); Serial.println(kecepatan);
      Serial.print("Kp: "); Serial.print(Kp);
      Serial.print("   Ki: "); Serial.print(Ki);
      Serial.print("   Kd: "); Serial.println(Kd);
      
      posisiSebelumnya = delta_posisi;
    }
    
  }

  void processHTTPRequest(WiFiClient &client) {
    if (header.indexOf("GET /set?") >= 0) {
      updatePIDParameters();
    }
    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.println("<link rel=\"icon\" href=\"data:,\">");
    client.println("<style>html { font-family: Helvetica; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; margin: 0; background-color: #f0f0f0;}");
    client.println(".section { background-color: #ffffff; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); width: 80%; max-width: 400px; margin-bottom: 20px; text-align: center;}");
    client.println("h1 { font-size: 24px; color: #333333; margin-bottom: 10px; } h2 { font-size: 20px; color: #555555; margin-bottom: 10px; }");
    client.println("input[type='number'] { font-size: 20px; padding: 10px; width: calc(100% - 20px); margin-bottom: 10px; border: 1px solid #cccccc; border-radius: 5px; }");
    client.println("input[type='submit'] { font-size: 20px; padding: 10px 20px; background-color: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; }");
    client.println("input[type='submit']:hover { background-color: #45a049; }");
    client.println("</style></head>");

    client.println("<body><form action='/set' method='GET'>");

    client.println("<div class='section'><h1>Kp</h1><h2>Current: " + String(Kp) + "</h2>");
    client.println("<input type='number' step='0.01' name='kp' value='" + String(Kp) + "'></div>");

    client.println("<div class='section'><h1>Ki</h1><h2>Current: " + String(Ki) + "</h2>");
    client.println("<input type='number' step='0.01' name='ki' value='" + String(Ki) + "'></div>");

    client.println("<div class='section'><h1>Kd</h1><h2>Current: " + String(Kd) + "</h2>");
    client.println("<input type='number' step='0.01' name='kd' value='" + String(Kd) + "'></div>");

    client.println("<div style='width: 100%; text-align: center;'><input type='submit' value='Set PID'></div>");

    client.println("</form></body></html>");
  }

  void updatePIDParameters() {
    int kpIndex = header.indexOf("kp=");
    int kiIndex = header.indexOf("ki=");
    int kdIndex = header.indexOf("kd=");
    if (kpIndex >= 0) Kp = header.substring(kpIndex + 3, header.indexOf("&", kpIndex)).toFloat();
    if (kiIndex >= 0) Ki = header.substring(kiIndex + 3, header.indexOf("&", kiIndex)).toFloat();
    if (kdIndex >= 0) Kd = header.substring(kdIndex + 3, header.indexOf(" ", kdIndex)).toFloat();
  }