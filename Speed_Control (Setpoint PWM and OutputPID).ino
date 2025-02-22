#include <driver/timer.h>
#include <WiFi.h>
#include <PID_v1_bc.h>

hw_timer_t* timer = NULL;
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

enum ControlMode { SPEED,
                   ]POSITION };
ControlMode controlMode = SPEED;

// PID Parameters
float Kp_speed = 1, Ki_speed = 0, Kd_speed = 0;
float Kp_pos = 0.5, Ki_pos = 0, Kd_pos = 0;

// System variables
volatile long posisi = 0;
float speed_setpoint = 0;  // Setpoint dalam RPM
float position_setpoint = 0;
float kecepatan_deg_per_sec = 0;  // Kecepatan dalam derajat/detik
float posisi_deg = 0;

volatile long posisiSebelumnya = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile long delta_posisi = 0;

volatile uint8_t encoderState = 0;  // FIX: Single encoder state tracker

// PID variables
float integral_speed = 0, errorSebelumnya_speed = 0;
float integral_pos = 0, errorSebelumnya_pos = 0;
float outputPID = 0;
int PWM = 0;

// Konversi RPM ke Derajat/detik
float rpmToDegSec(float rpm) {
  return rpm * 6.0;  // 1 RPM = 6 derajat/detik
}

// Konversi Derajat/detik ke RPM
float degSecToRpm(float deg_sec) {
  return deg_sec / 6.0;
}

void IRAM_ATTR handleEncoder() {
  static uint8_t oldState = 0;
  uint8_t newState = (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);
  uint8_t transition = (oldState << 2) | newState;

  portENTER_CRITICAL_ISR(&mux);
  if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000) {
    posisi--;
  } else if (transition == 0b0010 || transition == 0b1011 || transition == 0b1101 || transition == 0b0100) {
    posisi++;
  }
  oldState = newState;
  portEXIT_CRITICAL_ISR(&mux);
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
  attachInterrupt(digitalPinToInterrupt(PIN_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), handleEncoder, CHANGE);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, []{ timerFlag = true; }, true);
  timerAlarmWrite(timer, 100000, true);  // Update interval 100ms
  timerAlarmEnable(timer);
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    processClientRequest(client);
  }

  if (timerFlag) {
    portENTER_CRITICAL(&timerMux);
    timerFlag = false;
    portEXIT_CRITICAL(&timerMux);

    calculateSystemVariables();
    runPIDController();
    updateMotorControl();

    Serial.print(controlMode == SPEED ? "SPEED |" : " POS |");
    Serial.print(" Setpoint:");
    Serial.print(controlMode == SPEED ? speed_setpoint : position_setpoint);
    Serial.print(" | Pos:");
    Serial.print(posisi_deg);
    Serial.print("° | Speed:");
    Serial.print(kecepatan_deg_per_sec);
    Serial.print("°/s | PIDout:");
    Serial.print(outputPID);
    Serial.print(" | PWM:");
    Serial.println(PWM);
  }
  handleSerialCommands();
}

// FIX: Atomic position reading
long getAtomicPosition() {
  long temp;
  portENTER_CRITICAL(&mux);
  temp = posisi;
  portEXIT_CRITICAL(&mux);
  return temp;
}  

//0000 = 0 ; 0001 = 1 ; 0010 = 2 ; 0011 = 3 ; 0100 = 4 ; 0101 = 5 ; 0110 = 6 ;
//0111 = 7 ; 1000 = 8 ; 1001 = 9 ; 1010 = 10 ; 1011 = 11 ; 1100 = 12 ;
//1101 = 13 ; 1110 = 14 ; 1111 = 15

void ISR_INT0() {
  // Encoder interrupt handler
  static uint8_t oldAB = 0;
  oldAB = ((oldAB << 2) | (digitalRead(PIN_A) << 1) | digitalRead(PIN_B)) & 0x0F;
  if (oldAB == 0b0001 || oldAB == 0b0111 || oldAB == 0b1110 || oldAB == 0b1000) posisi--;
  else if (oldAB == 0b0010 || oldAB == 0b1011 || oldAB == 0b1101 || oldAB == 0b0100) posisi++;
}

void ISR_INT1() {
  // Encoder interrupt handler
  static uint8_t oldAB = 0;
  oldAB = ((oldAB << 2) | (digitalRead(PIN_A) << 1) | digitalRead(PIN_B)) & 0x0F;
  if (oldAB == 0b0001 || oldAB == 0b0111 || oldAB == 0b1110 || oldAB == 0b1000) posisi++;
  else if (oldAB == 0b0010 || oldAB == 0b1011 || oldAB == 0b1101 || oldAB == 0b0100) posisi--;
}

void calculateSystemVariables() {
  const float degrees_per_count = ((1 / 4) * 360 / PPR);
  unsigned long timer_period_us = timerAlarmRead(timer);

  posisi_deg = (int)(posisi * degrees_per_count) % 360;

  // Hitung kecepatan dalam derajat/detik
  long delta_counts = posisi - posisiSebelumnya;
  kecepatan_deg_per_sec = delta_counts * (1000000.0 / timer_period_us) * degrees_per_count;

  posisiSebelumnya = posisi;
}

void runPIDController() {
  if(controlMode == SPEED) {
    float error = speed_setpoint - kecepatan_deg_per_sec;
    integral_speed += error * 0.1;  // 100ms dt
    integral_speed = constrain(integral_speed, -255, 255);
    float derivative = (error - errorSebelumnya_speed) / 0.1;
    outputPID = Kp_speed*error + Ki_speed*integral_speed + Kd_speed*derivative;
    errorSebelumnya_speed = error;
  } else {
    float error = position_setpoint - posisi_deg;
    integral_pos += error * 0.1;  // 100ms dt
    integral_pos = constrain(integral_pos, -1000, 1000);
    float derivative = (error - errorSebelumnya_pos) / 0.1;
    outputPID = Kp_pos*error + Ki_pos*integral_pos + Kd_pos*derivative;
    errorSebelumnya_pos = error;
  }
  outputPID = constrain(outputPID, -255, 255);
}

void updateMotorControl() {
  int deadzone = 10;  // Adjust based on motor characteristics
  PWM = abs(outputPID) < deadzone ? 0 : constrain(abs(outputPID), 0, 255);
  
  if(outputPID > 0) {
    analogWrite(RPWM_PIN, PWM);
    analogWrite(LPWM_PIN, 0);
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, PWM);
  }
}

void handleSerialCommands() {
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if(cmd.startsWith("SP ")) {
      float sp = cmd.substring(3).toFloat();
      if(controlMode == SPEED) speed_setpoint = sp;
      else position_setpoint = sp;
      Serial.print("Setpoint updated: ");
      Serial.println(sp);
    }
    else if(cmd.startsWith("KP ")) {
      float kp = cmd.substring(3).toFloat();
      if(controlMode == SPEED) Kp_speed = kp;
      else Kp_pos = kp;
      Serial.print("KP updated: ");
      Serial.println(kp);
    }
    else if(cmd.startsWith("KI ")) {
      float ki = cmd.substring(3).toFloat();
      if(controlMode == SPEED) Ki_speed = ki;
      else Ki_pos = ki;
      Serial.print("KI updated: ");
      Serial.println(ki);
    }
    else if(cmd.startsWith("KD ")) {
      float kd = cmd.substring(3).toFloat();
      if(controlMode == SPEED) Kd_speed = kd;
      else Kd_pos = kd;
      Serial.print("KD updated: ");
      Serial.println(kd);
    }
    else if(cmd.startsWith("MODE ")) {
      if(cmd.substring(5) == "SPEED") controlMode = SPEED;
      else if(cmd.substring(5) == "POS") controlMode = POSITION;
      Serial.print("Mode changed to: ");
      Serial.println(controlMode == SPEED ? "SPEED" : "POSITION");
    }
  }
}

  void processClientRequest(WiFiClient & client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            sendHTMLResponse(client);
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

  void sendHTMLResponse(WiFiClient & client) {
    parseHTTPParameters();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.println("<style>");
    client.println("body { font-family: Arial; margin: 20px; }");
    client.println(".container { max-width: 600px; margin: auto; }");
    client.println(".section { margin-bottom: 20px; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }");
    client.println("input, select { margin: 5px 0; width: 40%; padding: 8px; }");
    client.println("</style></head>");

    client.println("<body><div class='container'>");
    client.println("<h1>Motor Control</h1>");

    client.println("<form action='/'>");

    // Control Mode Selection
    client.println("<div class='section'>");
    client.println("<h2>Control Mode</h2>");
    client.println("<select name='mode'>");
    client.print("<option value='speed' ");
    if (controlMode == SPEED) client.print("selected");
    client.println(">Speed Control</option>");
    client.print("<option value='position' ");
    if (controlMode == POSITION) client.print("selected");
    client.println(">Position Control</option>");
    client.println("</select></div>");

    // Setpoint Input
    client.println("<div class='section'>");
    client.println("<h2>Setpoint</h2>");
    client.print("<input type='number' name='setpoint' step='0.1' value='");
    client.print(controlMode == SPEED ? speed_setpoint : position_setpoint);
    client.println("'>");

    // PID Parameters
    client.println("<div class='section'>");
    client.println("<h2>PID Parameters</h2>");
    client.println("<h3>Speed Control</h3>");
    client.println("Kp: <input type='number' step='0.01' name='kp_speed' value='" + String(Kp_speed) + "'>");
    client.println("Ki: <input type='number' step='0.01' name='ki_speed' value='" + String(Ki_speed) + "'>");
    client.println("Kd: <input type='number' step='0.01' name='kd_speed' value='" + String(Kd_speed) + "'>");

    client.println("<h3>Position Control</h3>");
    client.println("Kp: <input type='number' step='0.01' name='kp_pos' value='" + String(Kp_pos) + "'>");
    client.println("Ki: <input type='number' step='0.01' name='ki_pos' value='" + String(Ki_pos) + "'>");
    client.println("Kd: <input type='number' step='0.01' name='kd_pos' value='" + String(Kd_pos) + "'>");
    client.println("</div>");

    client.println("<input type='submit' value='Update Settings'>");
    client.println("</form></div></body></html>");
  }

  void parseHTTPParameters() {
    if (header.indexOf("GET /?") >= 0) {
      // Parse control mode
      if (header.indexOf("mode=speed") != -1) controlMode = SPEED;
      if (header.indexOf("mode=position") != -1) controlMode = POSITION;

      // Parse setpoint
      int setpointIndex = header.indexOf("setpoint=");
      if (setpointIndex != -1) {
        float sp = header.substring(setpointIndex + 9, header.indexOf("&", setpointIndex)).toFloat();
        if (controlMode == SPEED) {
          speed_setpoint = sp;  // Simpan sebagai RPM
        } else {
          position_setpoint = sp;
        }
      }

      // Parse PID parameters
      Kp_speed = getParamValue("kp_speed=", Kp_speed);
      Ki_speed = getParamValue("ki_speed=", Ki_speed);
      Kd_speed = getParamValue("kd_speed=", Kd_speed);

      Kp_pos = getParamValue("kp_pos=", Kp_pos);
      Ki_pos = getParamValue("ki_pos=", Ki_pos);
      Kd_pos = getParamValue("kd_pos=", Kd_pos);
    }
  }

  float getParamValue(String param, float defaultValue) {
    int index = header.indexOf(param);
    if (index == -1) return defaultValue;
    int endIndex = header.indexOf("&", index);
    if (endIndex == -1) endIndex = header.indexOf(" ", index);
    return header.substring(index + param.length(), endIndex).toFloat();
  }
