#include <esp_now.h>
#include <WiFi.h>

#define Front_Right_Forward 
#define Front_Right_Back
#define Front_Back_Forward
// Structure example to receive data
// Must match the sender structure

// --- PIN DEFINITIONS ---

// --- LEFT SIDE (Motors 1 & 2) ---
// Motor 1
const int M1A = 32;
const int M1B = 33;
const int EN1 = 4;   // Connect to Driver 1 ENA

// Motor 2
const int M2A = 25;
const int M2B = 26;
const int EN2 = 13;  // Connect to Driver 1 ENB

// --- RIGHT SIDE (Motors 3 & 4) ---
// Motor 3
const int M3A = 19;
const int M3B = 21;
const int EN3 = 5;  // Connect to Driver 2 ENA (Avoid Pin 34-39)

// Motor 4
const int M4A = 22;
const int M4B = 23;
const int EN4 = 18;  // Connect to Driver 2 ENB

// Global Speed Variable (0-255)
int moveSpeed = 180; 


typedef struct struct_message {
  int id;
  int x_angle;
  int y_angle;
  int z_angle;
}struct_message;

// Create a struct_message called myData
struct_message myData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  Serial.printf("x value: %d \n", myData.x_angle);
  Serial.printf("y value: %d \n", myData.y_angle);
  Serial.printf("z value: %d \n", myData.z_angle);
  Serial.println();
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(M1A, OUTPUT); 
  pinMode(M1B, OUTPUT);

  pinMode(M2A, OUTPUT); 
  pinMode(M2B, OUTPUT);
  
  pinMode(M3A, OUTPUT); 
  pinMode(M3B, OUTPUT);

  pinMode(M4A, OUTPUT); 
  pinMode(M4B, OUTPUT);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);

  stopMotors();

  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

  // Move Forward if X is greater than 100

  if(myData.x_angle > 75){
    moveForward(moveSpeed);
  }
  else if(myData.x_angle < -75){
    moveBackward(moveSpeed);
  }
  else if(myData.y_angle > 75)
  {
    moveLeft(moveSpeed);
  }
  else if(myData.y_angle < -75)
  {
    moveRight(moveSpeed);
  }
  else
  {
    stopMotors();
  }
  


  // Acess the  variables for each board
  /*int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;*/
   // TEST SEQUENCE
  /*
  // 1. Move Forward for 2 seconds
  Serial.println("Forward");
  moveForward(moveSpeed);
  delay(2000);

  // 2. Stop for 1 second
  
  stopMotors();
  delay(1000);

  // 3. Move Backward for 2 seconds
  Serial.println("Backwards");
  moveBackward(moveSpeed);
  delay(2000);
  */
  // 4. Turn Left (Tank Turn) for 1 second
  /*
  Serial.println("Left");
  moveLeft(moveSpeed);
  delay(2000);

  stopMotors();
  delay(1000);

  // 5. Turn Right (Tank Turn) for 1 second
  Serial.println("Right");
  moveRight(moveSpeed);
  delay(2000);
  
  stopMotors();
  delay(1000);
  */

}

void moveForward(int speed) {
  // Set Directions
  // Adjust HIGH/LOW if a wheel spins backward
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);

  digitalWrite(M2A, HIGH); 
  digitalWrite(M2B, LOW);

  digitalWrite(M3A, HIGH); 
  digitalWrite(M3B, LOW);

  digitalWrite(M4A, HIGH); 
  digitalWrite(M4B, LOW);

  // Set Speed
  setSpeed(speed);
}

void moveBackward(int speed) {
  // Reverse Directions
  digitalWrite(M1A, LOW); 
  digitalWrite(M1B, HIGH);

  digitalWrite(M2A, LOW); 
  digitalWrite(M2B, HIGH);

  digitalWrite(M3A, LOW); 
  digitalWrite(M3B, HIGH);

  digitalWrite(M4A, LOW); 
  digitalWrite(M4B, HIGH);

  setSpeed(speed);
}

void moveRight(int speed) {
  // Left side moves BACK, Right side moves FORWARD
  digitalWrite(M1A, LOW);  
  digitalWrite(M1B, HIGH); 

  digitalWrite(M2A, HIGH);  
  digitalWrite(M2B, LOW);

  digitalWrite(M3A, HIGH); 
  digitalWrite(M3B, LOW);

  digitalWrite(M4A, LOW); 
  digitalWrite(M4B, HIGH);

  setSpeed(speed);
}

void moveLeft(int speed) {
  // Left side moves FORWARD, Right side moves BACK
  digitalWrite(M1A, HIGH); 
  digitalWrite(M1B, LOW);

  digitalWrite(M2A, LOW); 
  digitalWrite(M2B, HIGH);

  digitalWrite(M3A, LOW);  
  digitalWrite(M3B, HIGH);

  digitalWrite(M4A, HIGH);  
  digitalWrite(M4B, LOW);

  setSpeed(speed);
}

void stopMotors() {
  // Set Speed to 0 (Efficient stop)
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  analogWrite(EN3, 0);
  analogWrite(EN4, 0);
}

void setSpeed(int speed) {
  // Writes the PWM value to all Enable pins
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  analogWrite(EN3, speed);
  analogWrite(EN4, speed);
}
