// /*********
//   Rui Santos
//   Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-dc-motor-l298n-motor-driver-control-speed-direction/
// *********/

// // Motor A
// int motor1Pin1 = 12; 
// int motor1Pin2 = 14; 
// int enable1Pin = 13; 

// // Setting minimum duty cycle
// int dutyCycle = 60;

// void setup() {
//   // sets the pins as outputs:
//   pinMode(motor1Pin1, OUTPUT);
//   pinMode(motor1Pin2, OUTPUT);
//   pinMode(enable1Pin, OUTPUT);

//   Serial.begin(115200);

//   // testing
//   Serial.print("Testing DC Motor...");
// }

// void loop() {

//   //Apply power to spin at maximum speed
//   digitalWrite(enable1Pin, HIGH);

//   // Move the DC motor forward at maximum speed
//   Serial.println("Moving Forward");
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH); 
//   delay(2000);

//   // Stop the DC motor
//   Serial.println("Motor stopped");
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, LOW);
//   delay(1000);

//   // Move DC motor backwards at maximum speed
//   Serial.println("Moving Backwards");
//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW); 
//   delay(2000);

//   // Stop the DC motor
//   Serial.println("Motor stopped");
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, LOW);
//   delay(1000);

//   // Move DC motor forward with increasing speed
//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW);
//   while (dutyCycle <= 255){
//     analogWrite(enable1Pin, dutyCycle);   
//     Serial.print("Forward with duty cycle: ");
//     Serial.println(dutyCycle);
//     dutyCycle = dutyCycle + 5;
//     delay(500);
//   }
//   dutyCycle = 60;
// }








// #include <Arduino.h>
// #include <ESP8266WiFi.h>
// #include <ArduinoWebsockets.h>
// #include <ArduinoJson.h>
// #include <Servo.h>

// // WiFi credentials
// const char* ssid = "ary"; // Your WiFi SSID
// const char* password = "okzf8690"; // Your WiFi password

// // WebSocket server details
// const char* ws_url = "wss://misskaurmotorserver.onrender.com"; // WebSocket URL

// // Motor control pins (L298N)
// #define ENA_MOTOR_A D2  // GPIO4 (D2) for ENA (speed, Motor A)
// #define IN1_MOTOR_A D1  // GPIO5 (D1) for IN1 (direction, Motor A)
// #define IN2_MOTOR_A D3  // GPIO0 (D3) for IN2 (direction, Motor A)
// #define ENB_MOTOR_B D6  // GPIO12 (D6) for ENB (speed, Motor B)
// #define IN3_MOTOR_B D5  // GPIO14 (D5) for IN3 (direction, Motor B)
// #define IN4_MOTOR_B D7  // GPIO13 (D7) for IN4 (direction, Motor B)

// // Servo pin
// #define SERVO_PIN D4 // GPIO2 (D4) for servo

// // State variables
// volatile int currentMovement = 0; // 1: forward sequence, -1: backward, 0: stop
// volatile bool currentToggle = false; // true: run servo, false: stop servo

// // Minimum duty cycle
// int dutyCycle = 60;

// Servo servo;
// websockets::WebsocketsClient client;

// // Motor control functions
// void stopMotors() {
//     digitalWrite(IN1_MOTOR_A, LOW);
//     digitalWrite(IN2_MOTOR_A, LOW);
//     digitalWrite(IN3_MOTOR_B, LOW);
//     digitalWrite(IN4_MOTOR_B, LOW);
//     analogWrite(ENA_MOTOR_A, 0); // 0% PWM to stop
//     analogWrite(ENB_MOTOR_B, 0); // 0% PWM to stop
//     Serial.println("Motors: STOP");
//     Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
//     Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
//     Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 0");
//     Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
//     Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
//     Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 0");
// }

// void motorsForward() {
//     digitalWrite(IN1_MOTOR_A, LOW);
//     digitalWrite(IN2_MOTOR_A, HIGH);
//     digitalWrite(IN3_MOTOR_B, LOW);
//     digitalWrite(IN4_MOTOR_B, HIGH);
//     analogWrite(ENA_MOTOR_A, 255); // Full speed
//     analogWrite(ENB_MOTOR_B, 255); // Full speed
//     Serial.println("Motors: FORWARD");
//     Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
//     Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
//     Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 255");
//     Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
//     Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
//     Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 255");
//     dutyCycle = 60;
// }

// void motorsBackward() {
//     digitalWrite(IN1_MOTOR_A, HIGH);
//     digitalWrite(IN2_MOTOR_A, LOW);
//     digitalWrite(IN3_MOTOR_B, HIGH);
//     digitalWrite(IN4_MOTOR_B, LOW);
//     analogWrite(ENA_MOTOR_A, 255); // Full speed
//     analogWrite(ENB_MOTOR_B, 255); // Full speed
//     Serial.println("Motors: BACKWARD");
//     Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
//     Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
//     Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 255");
//     Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
//     Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
//     Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 255");
// }

// void motorsForwardIncreasingSpeed() {
//     digitalWrite(IN1_MOTOR_A, LOW);
//     digitalWrite(IN2_MOTOR_A, HIGH);
//     digitalWrite(IN3_MOTOR_B, LOW);
//     digitalWrite(IN4_MOTOR_B, HIGH);
//     while (dutyCycle <= 255) {
//         analogWrite(ENA_MOTOR_A, dutyCycle);
//         analogWrite(ENB_MOTOR_B, dutyCycle);
//         Serial.print("Forward with duty cycle: ");
//         Serial.println(dutyCycle);
//         dutyCycle += 5;
//         delay(250);
//     }
//     // dutyCycle = 60; // Reset duty cycle
// }

// // Servo control function
// void controlServo() {
//     if (!currentToggle) {
//         servo.write(0); // Stop at 0 degrees
//         Serial.println("Servo: STOP (0 degrees)");
//     } else {
//         servo.write(30); // Move to 30 degrees
//         Serial.println("Servo: MOVE (30 degrees)");
//     }
// }

// // Handle state changes
// void handleState() {
//     if (currentMovement == 1) {
//         motorsForwardIncreasingSpeed();
//         motorsForward();
//     } else if (currentMovement == -1) {
//         motorsBackward();
//     } else {
//         stopMotors();
//     }
// }

// // WebSocket message callback
// void onMessageCallback(websockets::WebsocketsMessage message) {
//     String data = message.data();
//     StaticJsonDocument<200> doc;
//     DeserializationError error = deserializeJson(doc, data);
//     if (error) {
//         Serial.print("JSON parse error: ");
//         Serial.println(error.c_str());
//         return;
//     }
//     currentMovement = doc["movement"];
//     currentToggle = doc["toggle"];
//     Serial.print("Received state -> Movement: ");
//     Serial.print(currentMovement);
//     Serial.print(", Toggle: ");
//     Serial.println(currentToggle);
//     handleState();
//     controlServo();
// }

// // WebSocket event callback
// void onEventsCallback(websockets::WebsocketsEvent event, String data) {
//     if (event == websockets::WebsocketsEvent::ConnectionOpened) {
//         Serial.println("WebSocket connected");
//     } else if (event == websockets::WebsocketsEvent::ConnectionClosed) {
//         Serial.println("WebSocket disconnected");
//     } else if (event == websockets::WebsocketsEvent::GotPing) {
//         Serial.println("Received PING");
//     } else if (event == websockets::WebsocketsEvent::GotPong) {
//         Serial.println("Received PONG");
//     }
// }

// // Setup function
// void setup() {
//     Serial.begin(115200);
//     Serial.println("ESP8266 starting...");

//     // Initialize motor pins
//     pinMode(ENA_MOTOR_A, OUTPUT);
//     pinMode(IN1_MOTOR_A, OUTPUT);
//     pinMode(IN2_MOTOR_A, OUTPUT);
//     pinMode(ENB_MOTOR_B, OUTPUT);
//     pinMode(IN3_MOTOR_B, OUTPUT);
//     pinMode(IN4_MOTOR_B, OUTPUT);

//     // Initialize servo
//     servo.attach(SERVO_PIN);
//     servo.write(0); // Start at 0 degrees

//     // Stop motors and servo initially
//     stopMotors();
//     controlServo();

//     // Connect to WiFi
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to WiFi (SSID: ");
//     Serial.print(ssid);
//     Serial.print(")");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("\nConnected to WiFi!");
//     Serial.print("IP Address: "); Serial.println(WiFi.localIP());
//     Serial.print("Signal Strength (RSSI): "); Serial.println(WiFi.RSSI());

//     // Setup WebSocket callbacks
//     client.onMessage(onMessageCallback);
//     client.onEvent(onEventsCallback);

//     // Connect to WebSocket server
//     bool connected = client.connect(ws_url);
//     if (connected) {
//         Serial.println("Connected to WebSocket server");
//     } else {
//         Serial.println("Failed to connect to WebSocket server");
//     }
// }

// // Loop function with reconnection logic
// unsigned long lastReconnectAttempt = 0;
// int reconnectDelay = 5000; // Start with 5 seconds

// void loop() {
//     client.poll();
//     if (!client.available()) {
//         unsigned long now = millis();
//         if (now - lastReconnectAttempt >= reconnectDelay) {
//             Serial.println("WebSocket disconnected, attempting to reconnect...");
//             if (client.connect(ws_url)) {
//                 Serial.println("Reconnected to WebSocket server");
//                 reconnectDelay = 5000; // Reset delay on success
//             } else {
//                 Serial.println("Reconnection failed");
//                 reconnectDelay = min(reconnectDelay * 2, 60000); // Double delay, max 60s
//             }
//             lastReconnectAttempt = now;
//         }
//     } else {
//         reconnectDelay = 5000; // Reset delay if connected
//     }
// }





#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <Servo.h>

// WiFi credentials
const char* ssid = "ary"; // Your WiFi SSID
const char* password = "okzf8690"; // Your WiFi password

// WebSocket server details
const char* ws_url = "wss://misskaurmotorserver.onrender.com"; // WebSocket URL

// Motor control pins (L298N)
#define ENA_MOTOR_A D2  // GPIO4 (D2) for ENA (speed, Motor A, left)
#define IN1_MOTOR_A D1  // GPIO5 (D1) for IN1 (direction, Motor A)
#define IN2_MOTOR_A D3  // GPIO0 (D3) for IN2 (direction, Motor A)
#define ENB_MOTOR_B D6  // GPIO12 (D6) for ENB (speed, Motor B, right)
#define IN3_MOTOR_B D5  // GPIO14 (D5) for IN3 (direction, Motor B)
#define IN4_MOTOR_B D7  // GPIO13 (D7) for IN4 (direction, Motor B)

// Servo pin
#define SERVO_PIN D4 // GPIO2 (D4) for servo

// State variables
volatile int currentMovement = 0; // 0: stop, 1: forward, -1: backward, 2: left, 3: right
volatile bool currentToggle = false; // true: run servo, false: stop servo

// Minimum duty cycle
int dutyCycle = 60;

Servo servo;
websockets::WebsocketsClient client;

// Motor control functions
void stopMotors() {
    digitalWrite(IN1_MOTOR_A, LOW);
    digitalWrite(IN2_MOTOR_A, LOW);
    digitalWrite(IN3_MOTOR_B, LOW);
    digitalWrite(IN4_MOTOR_B, LOW);
    analogWrite(ENA_MOTOR_A, 0); // 0% PWM to stop
    analogWrite(ENB_MOTOR_B, 0); // 0% PWM to stop
    Serial.println("Motors: STOP");
    Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
    Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
    Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 0");
    Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
    Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
    Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 0");
}

void motorsForward() {
    digitalWrite(IN1_MOTOR_A, LOW);
    digitalWrite(IN2_MOTOR_A, HIGH);
    digitalWrite(IN3_MOTOR_B, LOW);
    digitalWrite(IN4_MOTOR_B, HIGH);
    analogWrite(ENA_MOTOR_A, 255); // Full speed
    analogWrite(ENB_MOTOR_B, 255); // Full speed
    Serial.println("Motors: FORWARD");
    Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
    Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
    Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 255");
    Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
    Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
    Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 255");
    dutyCycle = 60;
}

void motorsBackward() {
    digitalWrite(IN1_MOTOR_A, HIGH);
    digitalWrite(IN2_MOTOR_A, LOW);
    digitalWrite(IN3_MOTOR_B, HIGH);
    digitalWrite(IN4_MOTOR_B, LOW);
    analogWrite(ENA_MOTOR_A, 255); // Full speed
    analogWrite(ENB_MOTOR_B, 255); // Full speed
    Serial.println("Motors: BACKWARD");
    Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
    Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
    Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 255");
    Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
    Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
    Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 255");
}

void motorsLeft() {
    digitalWrite(IN1_MOTOR_A, HIGH); // Left motor backward
    digitalWrite(IN2_MOTOR_A, LOW);
    digitalWrite(IN3_MOTOR_B, LOW);  // Right motor forward
    digitalWrite(IN4_MOTOR_B, HIGH);
    analogWrite(ENA_MOTOR_A, 200); // Reduced speed for turning
    analogWrite(ENB_MOTOR_B, 200);
    Serial.println("Motors: LEFT");
    Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
    Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
    Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 200");
    Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
    Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
    Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 200");
}

void motorsRight() {
    digitalWrite(IN1_MOTOR_A, LOW);  // Left motor forward
    digitalWrite(IN2_MOTOR_A, HIGH);
    digitalWrite(IN3_MOTOR_B, HIGH); // Right motor backward
    digitalWrite(IN4_MOTOR_B, LOW);
    analogWrite(ENA_MOTOR_A, 200); // Reduced speed for turning
    analogWrite(ENB_MOTOR_B, 200);
    Serial.println("Motors: RIGHT");
    Serial.print("IN1_MOTOR_A: "); Serial.println(digitalRead(IN1_MOTOR_A));
    Serial.print("IN2_MOTOR_A: "); Serial.println(digitalRead(IN2_MOTOR_A));
    Serial.print("ENA_MOTOR_A: "); Serial.println("PWM 200");
    Serial.print("IN3_MOTOR_B: "); Serial.println(digitalRead(IN3_MOTOR_B));
    Serial.print("IN4_MOTOR_B: "); Serial.println(digitalRead(IN4_MOTOR_B));
    Serial.print("ENB_MOTOR_B: "); Serial.println("PWM 200");
}

void motorsForwardIncreasingSpeed() {
    digitalWrite(IN1_MOTOR_A, LOW);
    digitalWrite(IN2_MOTOR_A, HIGH);
    digitalWrite(IN3_MOTOR_B, LOW);
    digitalWrite(IN4_MOTOR_B, HIGH);
    while (dutyCycle <= 255) {
        analogWrite(ENA_MOTOR_A, dutyCycle);
        analogWrite(ENB_MOTOR_B, dutyCycle);
        Serial.print("Forward with duty cycle: ");
        Serial.println(dutyCycle);
        dutyCycle += 5;
        delay(250);
    }
}

// Servo control function
void controlServo() {
    if (!currentToggle) {
        servo.write(0); // Stop at 0 degrees
        Serial.println("Servo: STOP (0 degrees)");
    } else {
        servo.write(30); // Move to 30 degrees
        Serial.println("Servo: MOVE (30 degrees)");
    }
}

// Handle state changes
void handleState() {
    switch (currentMovement) {
        case 1:
            motorsForwardIncreasingSpeed();
            motorsForward();
            break;
        case -1:
            motorsBackward();
            break;
        case 2:
            motorsLeft();
            break;
        case 3:
            motorsRight();
            break;
        default:
            stopMotors();
            break;
    }
}

// WebSocket message callback
void onMessageCallback(websockets::WebsocketsMessage message) {
    String data = message.data();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
    }
    currentMovement = doc["movement"];
    currentToggle = doc["toggle"];
    Serial.print("Received state -> Movement: ");
    Serial.print(currentMovement);
    Serial.print(", Toggle: ");
    Serial.println(currentToggle);
    handleState();
    controlServo();
}

// WebSocket event callback
void onEventsCallback(websockets::WebsocketsEvent event, String data) {
    if (event == websockets::WebsocketsEvent::ConnectionOpened) {
        Serial.println("WebSocket connected");
    } else if (event == websockets::WebsocketsEvent::ConnectionClosed) {
        Serial.println("WebSocket disconnected");
    } else if (event == websockets::WebsocketsEvent::GotPing) {
        Serial.println("Received PING");
    } else if (event == websockets::WebsocketsEvent::GotPong) {
        Serial.println("Received PONG");
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    Serial.println("ESP8266 starting...");

    // Initialize motor pins
    pinMode(ENA_MOTOR_A, OUTPUT);
    pinMode(IN1_MOTOR_A, OUTPUT);
    pinMode(IN2_MOTOR_A, OUTPUT);
    pinMode(ENB_MOTOR_B, OUTPUT);
    pinMode(IN3_MOTOR_B, OUTPUT);
    pinMode(IN4_MOTOR_B, OUTPUT);

    // Initialize servo
    servo.attach(SERVO_PIN);
    servo.write(0); // Start at 0 degrees

    // Stop motors and servo initially
    stopMotors();
    controlServo();

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi (SSID: ");
    Serial.print(ssid);
    Serial.print(")");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): "); Serial.println(WiFi.RSSI());

    // Setup WebSocket callbacks
    client.onMessage(onMessageCallback);
    client.onEvent(onEventsCallback);

    // Connect to WebSocket server
    bool connected = client.connect(ws_url);
    if (connected) {
        Serial.println("Connected to WebSocket server");
    } else {
        Serial.println("Failed to connect to WebSocket server");
    }
}

// Loop function with reconnection logic
unsigned long lastReconnectAttempt = 0;
int reconnectDelay = 5000; // Start with 5 seconds

void loop() {
    client.poll();
    if (!client.available()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt >= reconnectDelay) {
            Serial.println("WebSocket disconnected, attempting to reconnect...");
            if (client.connect(ws_url)) {
                Serial.println("ReMBconnected to WebSocket server");
                reconnectDelay = 5000; // Reset delay on success
            } else {
                Serial.println("Reconnection failed");
                reconnectDelay = min(reconnectDelay * 2, 60000); // Double delay, max 60s
            }
            lastReconnectAttempt = now;
        }
    } else {
        reconnectDelay = 5000; // Reset delay if connected
    }
}