#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xB1, 0xE2, 0x2C};

// Structure example to send data
// Must match the receiver structure exactly
typedef struct struct_message {
  int id; // must be unique for each sender board
  int x_angle;
  int y_angle;
  int z_angle;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status != 0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // calibrate gyro and accelero
  Serial.println("Done!\n");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback to get the status of transmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // mpu.update() MUST run as fast as possible to keep angle calculations accurate
  mpu.update();

  // Non-blocking timer: Send data every 1000ms (1 second)
  if ((millis() - timer) > 1000) {
    
    // Set values to send (casting floats to ints to match your struct)
    myData.id = 1;
    myData.x_angle = (int)mpu.getAngleX();
    myData.y_angle = (int)mpu.getAngleY();
    myData.z_angle = (int)mpu.getAngleZ();

    // Print to Serial Monitor for debugging
    Serial.print("X: "); Serial.print(myData.x_angle);
    Serial.print("\tY: "); Serial.print(myData.y_angle);
    Serial.print("\tZ: "); Serial.println(myData.z_angle);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success\n");
    } else {
      Serial.println("Error sending the data\n");
    }
    
    // Reset the timer
    timer = millis();
  }
}