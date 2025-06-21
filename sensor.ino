#include <base64.hpp>

#include <ESP8266WiFi.h>       // For WiFi connectivity
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>


#include <LittleFS.h>
#include <CRC.h>
#include <Wire.h>
#include <RTClib.h>


#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_ADS1X15.h>
//#include <INA228.h>
#include <INA226.h>
#define DEBUG_ESP_WIFI false
#define DEBUG_ESP_PORT Serial // Change to your preferred Serial port
#define DEBUG_ESP_HTTP_CLIENT true
#define DEBUG_ESP_HTTP_CLIENT true

#define RTC_ADDRESS 0x51


// WiFi and MQTT clients
WiFiClient espClient;
BearSSL::WiFiClientSecure *espClientSecure;
//BearSSL::CertStore certStore; // Unused object

const char* host = "raw.githubusercontent.com";
const int httpsPort = 443;
// Define firmware URLs
//#define sensor_firmware_version_url "https://google.co.uk"
#define sensor_firmware_version_url "https://raw.githubusercontent.com/thermalsystemsltd/sensorfirmware/refs/heads/main/version"
#define sensor_firmware_bin_url "https://github.com/thermalsystemsltd/sensorfirmware/blob/main/sensorfirmware.bin"
const String firmwareBinURL = "https://raw.githubusercontent.com/thermalsystemsltd/sensorfirmware/main/sensorfirmware.bin";

// Trust Root
const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEyDCCA7CgAwIBAgIQDPW9BitWAvR6uFAsI8zwZjANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0yMTAzMzAwMDAwMDBaFw0zMTAzMjkyMzU5NTlaMFkxCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxMzAxBgNVBAMTKkRpZ2lDZXJ0IEdsb2Jh
bCBHMiBUTFMgUlNBIFNIQTI1NiAyMDIwIENBMTCCASIwDQYJKoZIhvcNAQEBBQAD
ggEPADCCAQoCggEBAMz3EGJPprtjb+2QUlbFbSd7ehJWivH0+dbn4Y+9lavyYEEV
cNsSAPonCrVXOFt9slGTcZUOakGUWzUb+nv6u8W+JDD+Vu/E832X4xT1FE3LpxDy
FuqrIvAxIhFhaZAmunjZlx/jfWardUSVc8is/+9dCopZQ+GssjoP80j812s3wWPc
3kbW20X+fSP9kOhRBx5Ro1/tSUZUfyyIxfQTnJcVPAPooTncaQwywa8WV0yUR0J8
osicfebUTVSvQpmowQTCd5zWSOTOEeAqgJnwQ3DPP3Zr0UxJqyRewg2C/Uaoq2yT
zGJSQnWS+Jr6Xl6ysGHlHx+5fwmY6D36g39HaaECAwEAAaOCAYIwggF+MBIGA1Ud
EwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFHSFgMBmx9833s+9KTeqAx2+7c0XMB8G
A1UdIwQYMBaAFE4iVCAYlebjbuYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAd
BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQG
CCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKG
NGh0dHA6Ly9jYWNlcnRzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RH
Mi5jcnQwQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29t
L0RpZ2lDZXJ0R2xvYmFsUm9vdEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwC
ATAHBgVngQwBATAIBgZngQwBAgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG
9w0BAQsFAAOCAQEAkPFwyyiXaZd8dP3A+iZ7U6utzWX9upwGnIrXWkOH7U1MVl+t
wcW1BSAuWdH/SvWgKtiwla3JLko716f2b4gp/DA/JIS7w7d7kwcsr4drdjPtAFVS
slme5LnQ89/nD/7d+MS5EHKBCQRfz5eeLjJ1js+aWNJXMX43AYGyZm0pGrFmCW3R
bpD0ufovARTFXFZkAdl9h6g4U5+LXUZtXMYnhIHUfoyMo5tS58aI7Dd8KvvwVVo4
chDYABPPTHPbqjc1qCmBaZx2vN4Ye5DUys/vZwP9BFohFrH/6j/f3IL16/RZkiMN
JCqVJUzKoZHm1Lesh3Sz8W2jmdv51b2EQJ8HmA==
-----END CERTIFICATE-----
)EOF";

X509List *cert;

#define RTC_SDA_PIN 4
#define RTC_SCL_PIN 5
#define EEPROM_ADDR 0x50
#define MAX_EEPROM_ADDRESS 8100
#define FIRMWARE_VERSION "1.0.8"
#define BUFFER_SIZE 512
#define DATA_FILE "/data.txt"
#define LAST_TIME_SYNC_FILE "/lastTimeSync.txt"
#define MAX_SYNC_RETRIES 3       // Maximum number of time sync retries
#define TIME_SYNC_INTERVAL 3600  // 1 hour in seconds
#define LAST_TIME_SYNC_FILE "/lastTimeSync.txt"

////////POWER SWITCHING///
#define THERMISTOR_POWER_PIN 9  // GPIO09 to control thermistor power
#define BATTERY_POWER_PIN 14    // GPIO14 to control battery power

#define LORA_SS_PIN 15          // GPIO15 (D8)
#define LORA_DIO0_PIN 0         // GPIO0 (D3 or RX)

#define LBT_RSSI_THRESHOLD -90  // RSSI threshold for clear channel (adjust as needed)
#define LBT_MAX_RETRIES 5       // Maximum number of retries for channel checking
#define LBT_BACKOFF_TIME 200    // Base backoff time in milliseconds



int loopCounter = 0;  // Counter to track loop cycles

// Initialize the ADS1115
Adafruit_ADS1115 *ads;  
INA226 *ina226;  // Use the correct I2C address, often 0x40

////////////////////////////////////////////////////
unsigned long bootTime = 0; // Global variable to store the time of boot.
bool timeSyncRequired = false;   // Declare the timeSyncRequired flag here globally
unsigned long lastTimeSync = 0;  // Will be updated from LittleFS
const int thermistorPin = A0;
const int loopDelay = 500;
String ackStatus = "N";
RTC_PCF8563 *rtc;
unsigned long lastLoRaTransmission = 0;
unsigned long loRaTransmissionDelay = 500;

String serialNumber = "131203";

String LoRaReadBuffer = "";
String SerialReadBuffer = "";

bool wrapEnabled = true;
bool ackReceived = false;
bool receivingFirmware = false;
bool updateRequired = false;


#define CRC_POLYNOMIAL 0xEDB88320
#define CRC_INITIAL_VALUE 0xFFFFFFFF


bool ackReceivedRecently = false;  // Add this line

enum DeviceState {
  NORMAL_OPERATION,
  UPDATE_MODE,
  RECEIVE_CHUNK,
  SEND_ACK
};

DeviceState currentState = NORMAL_OPERATION;

std::vector<int> parseVersionString(const String& version);
String cleanIncomingData(const String& rawData);  // Updated to pass by const reference
void clearFile();
void handleSerialCommands();
void setTimeFromSerial();
float getTemperature();
bool writeToFile(float temperature, unsigned long timestamp, const String& ackStatus);
void readFromFile();
void printDateTime(DateTime dt);
void printTwoDigits(int number);
void processLoRaMessage(const String& message);
bool isValidMessageFormat(const String& message);
void sendACK(int chunkNumber);
void resendLoggedData();
void initializeWritePosition();
unsigned long readLastTimeSync();
void testBinaryUrl();


///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////

void setup() {
  WiFi.forceSleepBegin();
  WiFi.mode(WIFI_OFF);  // Optional redundancy
  delay(1);             // Give time for shutdown
  system_update_cpu_freq(40); // Set CPU to 40 MHz
  
  Serial.printf("Free Heap: %d\n", ESP.getFreeHeap());

  // Initialize the certificate
  cert = new X509List(trustRoot);
  if (cert == nullptr) {
    Serial.println("Failed to allocate memory for X509List!");
    while(1) delay(100);
  }
  yield();

  // GPIO10 setup
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW); // Ensure GPIO10 starts low at power-up


if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS. Cannot proceed.");
    while (1); // Halt if mounting fails
}
yield();
  // Start serial communication for debugging
  Serial.begin(115200);
  bootTime = millis(); // Record the boot time
 // delay(20);  // Give time for the ESP to stabilize after waking
 // Serial.println("Booting");
  //delay(20);

  

  // Reinitialize I2C bus
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);  // SDA and SCL pin for RTC
  //Serial.println("I2C bus reinitialized.");

  // Initialize the RTC
  rtc = new RTC_PCF8563();
  if (rtc == nullptr) {
    Serial.println("Failed to allocate memory for RTC!");
    while(1) delay(100);
  }
  rtc->begin();
  yield();
  //Serial.println("RTC initialized.");

  // Test RTC communication
  if (!Wire.requestFrom(0x51, 7)) {  // 0x51 is the address of PCF8563
    Serial.println("Failed to communicate with the RTC over I2C.");
  } else {
    Serial.println("RTC communication successful.");
  }
  yield();






  // Initialize ADS1115
  ads = new Adafruit_ADS1115();
  if (ads == nullptr) {
    Serial.println("Failed to allocate memory for ADS1115!");
    while(1) delay(100);
  }
  if (!ads->begin()) {
    Serial.println("Failed to initialize ADS1115");
    while (1);
  }
  ads->setGain(GAIN_ONE); // Set gain for ADS1115
  yield();

  // Initialize the INA226
  ina226 = new INA226(0x41);
  if (ina226 == nullptr) {
    Serial.println("Failed to allocate memory for INA226!");
    while(1) delay(100);
  }
  if (!ina226->begin()) {
    Serial.println("Failed to initialize INA226!");
    while (1);  // Halt if INA226 fails
  }
  yield();


// Attempt to read serial number from LittleFS
    String storedSerialNumber = readSerialNumberFromLittleFS();
    yield();
    if (!storedSerialNumber.isEmpty()) {
        serialNumber = storedSerialNumber.c_str();  // Override the hardcoded serial number
        Serial.println("Overridden hardcoded serial number with: " + storedSerialNumber);
    } else {
        //Serial.println("Using hardcoded serial number: " + String(serialNumber));
    }





  // Initialize LoRa settings
  LoRa.setPins(LORA_SS_PIN, LORA_DIO0_PIN);  // SS and DIO0 pin setup
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(125E3);  // 125 kHz
  LoRa.setCodingRate4(5);          // Coding Rate 4/5
  LoRa.setTxPower(1, PA_OUTPUT_PA_BOOST_PIN);  // Set power to 10 dBm (10 mW) using PA_BOOST

  if (!LoRa.begin(433E6)) {  // Set frequency to 433 MHz
    Serial.println("Starting LoRa failed!");
    while (1);  // Stop further execution if LoRa fails
  }

  // Check if RTC lost power
  if (rtc->lostPower()) {
    Serial.println("RTC is not running! Setting time sync required.");
    timeSyncRequired = true;
  } else {
    // Log RTC time at boot
    DateTime bootTime = rtc->now();
    Serial.print("RTC From Boot: ");
    printDateTime(bootTime);

    // Initialize LittleFS and format if needed
    if (!LittleFS.begin()) {
        Serial.println("Failed to mount LittleFS, formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("Failed to mount LittleFS after formatting");
            return;
        }
    }

    // Check LittleFS for last valid time
    File file = LittleFS.open("/lastValidTime.txt", "r");
    yield();
    if (file) {
        String savedTimeStr = file.readStringUntil('\n');
        unsigned long savedTime = savedTimeStr.toInt();
        if (savedTime > 0) {
            DateTime littlefsTime(savedTime);
            Serial.println("adjust RTC via LittleFS: ");
            printDateTime(littlefsTime);

            // Calculate time difference between RTC boot time and LittleFS time
            unsigned long timeDifference = abs((long)(bootTime.unixtime() - littlefsTime.unixtime()));

            if (timeDifference > 300) {  // 5 minutes in seconds
                Serial.println("Significant time difference detected. Setting time sync required.");
                timeSyncRequired = true;
            } else {
                Serial.println(" No significant difference detected.");
            }
        } else {
            Serial.println("No valid time in LittleFS; time sync required.");
            timeSyncRequired = true;
        }
        file.close();
    } else {
        Serial.println("No saved time in LittleFS; time sync required.");
        timeSyncRequired = true;
    }
  }




////////////////////////////////////  /////////////////////////////////////////END RTC TEST ALRMS


  // Reserve memory for buffers
  LoRaReadBuffer.reserve(256);
  SerialReadBuffer.reserve(256);

  // Log firmware version
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);
//////////////////////////////////////////////////////////////////////////

// Initialize BearSSL Secure Client
  espClientSecure = new BearSSL::WiFiClientSecure();
  if (espClientSecure == nullptr) {
    Serial.println("Failed to allocate memory for BearSSLClient!");
    while(1) delay(100);
  }

  espClientSecure->setInsecure(); // Temporarily bypass certificate validation for debugging
  espClientSecure->setTimeout(15000); // Set a timeout of 15 seconds


}


/////END SETUP ////

// Helper function to print DateTime
void printDateTime(DateTime dt) {
  Serial.print(dt.year());
  Serial.print("-");
  printTwoDigits(dt.month());
  Serial.print("-");
  printTwoDigits(dt.day());
  Serial.print(" ");
  printTwoDigits(dt.hour());
  Serial.print(":");
  printTwoDigits(dt.minute());
  Serial.print(":");
  printTwoDigits(dt.second());
}

// Helper function to print two digits
void printTwoDigits(int number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

// Helper function to convert Unix time to DateTime
DateTime unixTimeToDateTime(unsigned long unixTime) {
  return DateTime(unixTime);  // DateTime class handles the conversion
}







void loop() {
    unsigned long loopStartTime = millis(); // Record the time at the start of the loop
    loopCounter++;  // Increment the loop counter (if needed for other purposes)
   // Serial.print("Loop counter: ");
    //Serial.println(loopCounter);
    //checkForFirmwareUpdate(); // This is now handled in the ACK/response processing section
    // Check for serial commands
    handleSerialCommands(); // Add this line to process serial commands
    
    
    //////////////////////////// TIME SYNC ////////////////////////////
    if (timeSyncRequired) {
        Serial.println("Time sync required. Sending time sync...");
        sendTimeSyncCommand();  // Send time sync request to the base station

        // Wait for the time sync response
        unsigned long syncWaitStartTime = millis();
        bool syncReceived = false;

        while (millis() - syncWaitStartTime < 200) {  // Wait up to 5 seconds for the response
            int packetSize = LoRa.parsePacket();
            if (packetSize > 0) {
                String response = "";
                while (LoRa.available()) {
                    response += (char)LoRa.read();
                }
                Serial.println("Received time sync response: " + response);
                
                String cleanedResponse = cleanIncomingData(response);
                if (cleanedResponse.startsWith("UPDATE:START")) {
                    handleFirmwareUpdate(cleanedResponse);
                } else {
                    processTimeSyncResponse(cleanedResponse);  // Process the time sync response
                }
                
                syncReceived = true;
                break;
            }
        }

        if (!syncReceived) {
            Serial.println("No time sync response received.");
        } else {
            // Save the last time sync to LittleFS
            writeLastTimeSync(rtc->now().unixtime());  // Record the current time as the last sync time
        }

        // Reset the time sync flag
        timeSyncRequired = false;

        // Directly proceed with temperature transmission after time sync
        Serial.println("Time sync completed. Proceeding with temperature transmission.");
    }

    //////////////////////////// TEMPERATURE MEASUREMENT /////////////////////////
    unsigned long currentMillis = millis();

    // Check if it's time to transmit temperature based on the loop or the recent time sync
    if (currentMillis - lastLoRaTransmission >= loRaTransmissionDelay || timeSyncRequired == false) {
        lastLoRaTransmission = currentMillis;
        //Serial.println("Starting temperature and voltage measurement...");

        // Get the temperature reading
        float temperature = getTemperature();
        Serial.print("Temperature reading: ");
        Serial.println(temperature);

        // Get the battery voltage
        float batteryVoltage = readBatteryVoltageINA();
        Serial.print("Battery Voltage: ");
        Serial.print(batteryVoltage, 2);
        Serial.println(" V");
/////////////////////////////////////////////////////////////NEW LBT CODE////////////////////////////////////////

       // Prepare the LoRa data payload
    String loRaData = "T:" + String(temperature, 2) + ",V:" + String(batteryVoltage, 2) + ",S:" + serialNumber + ",FW:" + FIRMWARE_VERSION;

    // Sanitize and calculate CRC for the LoRa message
    String sanitizedLoRaData = sanitizeData(loRaData);
    uint32_t crcValue = crc32((const uint8_t*)sanitizedLoRaData.c_str(), sanitizedLoRaData.length());
    String crcString = String(crcValue, HEX);

    // Final LoRa packet with CRC and data
    loRaData = crcString + "," + loRaData;
   // Serial.print("Prepared Data: " + loRaData);
   // Serial.println();
    // Implement Listen Before Talk (LBT)
    int retries = 0;
    bool channelClear = false;

    while (retries < LBT_MAX_RETRIES) {
        if (isChannelClear()) {
            channelClear = true;
            break;
        }
        // If the channel is busy, wait for a random backoff period
        int backoff = LBT_BACKOFF_TIME + random(0, LBT_BACKOFF_TIME);
        Serial.println("Channel busy. Waiting for ");
        Serial.print(backoff);
        Serial.println(" ms");
        delay(backoff);
        retries++;
    }

    if (channelClear) {
        //Serial.println("Channel is clear. Transmitting data...");
        LoRa.beginPacket();
        LoRa.print(loRaData);
        LoRa.endPacket();
        Serial.println("Data transmitted successfully.");
    } else {
        Serial.println("Failed to transmit data. Channel is busy after maximum retries.");
    }

///////////////////////////////////////////////////////////////NEW LBT CODE ///////////////////////////////////////////
        //Serial.println("Checking for incoming ACK...");

        // Non-blocking ACK waiting
        unsigned long ackWaitStartTime = millis();
        ackReceived = false;

        while (millis() - ackWaitStartTime < 300) {
            yield();  // Allow background tasks to run to avoid WDT reset

            // Check if ACK or other packet received
            int ackPacketSize = LoRa.parsePacket();
            if (ackPacketSize > 0) {
                String loRaResponse = "";
                while (LoRa.available()) {
                    loRaResponse += (char)LoRa.read();
                }
                //Serial.println("LoRa Response: " + loRaResponse);

                String trimmedLoRaResponse = cleanIncomingData(loRaResponse);

                // Process ACK
                if (trimmedLoRaResponse.endsWith("ACK") && isValidMessageFormat(trimmedLoRaResponse)) {
                    ackStatus = "y";
                    ackReceived = true;
                    ackReceivedRecently = true;
                    Serial.println("ACK received successfully.");
                    checkForBackfill();
                    break;
                } else if (trimmedLoRaResponse.endsWith("ACK:N")) {
                    ackStatus = "n";
                    ackReceived = false;
                    Serial.println("Negative ACK received.");
                    break;
                } else if (trimmedLoRaResponse.startsWith("UPDATE:START")) {
                    handleFirmwareUpdate(trimmedLoRaResponse);
                } else {
                    ackStatus = "?";
                    Serial.println("Unrecognized LoRa message format: " + trimmedLoRaResponse);
                }
            }
        }

        if (!ackReceived) {
            Serial.println("No ACK received within timeout.");
            ackStatus = "n";
        }

        if (!writeToFile(temperature, rtc->now().unixtime(), ackStatus)) {
            Serial.println("Failed to write data to file!");
        }
    } else {
        Serial.println("Temperature transmission not triggered yet.");
    }

    //////////////////////////// POWER OFF & TIMER SIGNAL ////////////////////////////

    LoRa.sleep(); // Forces module to low-power mode


    pinMode(10, OUTPUT); // Ensure GPIO10 is set as an output
    digitalWrite(10, HIGH); // Set GPIO10 high to signal "done" to the TPL5110 timer  
    Serial.println("Hold GPIO HIGH"); 
    delay(200); // Hold signal for a short period
    //digitalWrite(10, LOW); // Optionally, reset GPIO10 to low
}





/////////////////////////////////////////////END MAIN LOOP/////////////////////////////

uint32_t calculateCRC(const String& message) {
  String sanitizedMessage = sanitizeData(message);
  return crc32((const uint8_t*)sanitizedMessage.c_str(), sanitizedMessage.length(), CRC_INITIAL_VALUE, CRC_POLYNOMIAL);
}

std::vector<int> parseVersionString(const String& version) {
  std::vector<int> versionParts;
  int start = 0;
  for (int i = 0; i < version.length(); ++i) {
    if (version[i] == '.' || version[i] == 'v') {
      versionParts.push_back(version.substring(start, i).toInt());
      start = i + 1;
    }
  }
  versionParts.push_back(version.substring(start).toInt());
  return versionParts;
}

void processLoRaMessage(const String& message) {
  Serial.println("Processing LoRa message: " + message);

  // Check if the message starts with the serial number followed by a comma
  if (message.startsWith(String(serialNumber) + ",")) {
    Serial.println("Serial number matches.");

    // Extract the datetime and update flag
    int firstCommaIndex = message.indexOf(",");
    int secondCommaIndex = message.indexOf(",", firstCommaIndex + 1);

    // Ensure we have valid comma positions
    if (firstCommaIndex != -1 && secondCommaIndex != -1) {
      String datetimeStr = message.substring(firstCommaIndex + 1, secondCommaIndex);  // Extract datetime string
      String updateFlag = message.substring(secondCommaIndex + 1);                    // Extract the "UPDATE" flag

      Serial.println("Extracted datetime: " + datetimeStr);
      Serial.println("Extracted flag: " + updateFlag);

      // Check if the message ends with "UPDATE"
      if (updateFlag == "UPDATE") {
        Serial.println("UPDATE flag detected. Processing time update...");

        // Parse the datetime string and update the RTC
        DateTime newDateTime = parseDateTime(datetimeStr);

        if (newDateTime.year() > 2000) {  // Basic validity check
          if (isBST(newDateTime)) {       // Adjust for BST after receiving time sync message
            //newDateTime = newDateTime + TimeSpan(0, 1, 0, 0);  // Add one hour
            Serial.println("BST is active. Adjusting time by +1 hour???");
          }
          rtc->adjust(newDateTime);  // Adjust RTC to new datetime
          //Serial.println("RTC updated with new datetime: " + datetimeStr);
        } else {
          Serial.println("Invalid datetime received.");
        }
      } else {
        Serial.println("No UPDATE flag detected.");
      }
    } else {
      Serial.println("Invalid message format (missing or misplaced commas).");
    }
  } else {
    Serial.println("Message does not match serial number.");
  }
}

bool isValidMessageFormat(const String& message) {
  int comma1Index = message.indexOf(',');
  int comma2Index = message.indexOf(',', comma1Index + 1);

  if (comma1Index != -1 && comma2Index != -1) {
    String crcString = message.substring(0, comma1Index);
    String receivedSerialNumber = message.substring(comma1Index + 1, comma2Index);
    String ackString = message.substring(comma2Index + 1);

    if (crcString.length() > 0 && receivedSerialNumber == serialNumber && ackString.startsWith("ACK")) {
      return true;
    }
  }

  Serial.println("Received non-ACK message or format mismatch: " + message);
  return false;
}

void sendACK(int chunkNumber) {
  String ackMessage = "ACK:" + String(chunkNumber) + "," + serialNumber + "\n";
  LoRa.beginPacket();
  LoRa.print(ackMessage);
  LoRa.endPacket();
  Serial.println("Sent ACK for chunk: " + String(chunkNumber));
  Serial.println("ACK Message: " + ackMessage);  // Print the ACK message being sent
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received command: ");
    Serial.println(command);

    if (command == "AT" || command == "AT+RX") {
      LoRa.beginPacket();
      LoRa.print(command);
      LoRa.endPacket();
      Serial.print("Command Executed");
    } else if (command == "loopforever") {
      wrapEnabled = !wrapEnabled;
      Serial.print("Circular buffer wrapping ");
      Serial.println(wrapEnabled ? "enabled" : "disabled");
    } else if (command == "read") {
      readFromFile();
    } else if (command == "clear") {
      clearFile();
      Serial.println("File cleared.");
    } else if (command == "settime") {
      setTimeFromSerial();
    } else if (command == "getdate") {
      DateTime now = rtc->now();
      Serial.print("Current Date and Time: ");
      printDateTime(now);
      Serial.println();
    } else if (command == "updatetime") {
      Serial.println("Triggering time sync...");
      timeSyncRequired = true;  // Trigger time sync via serial command
    } else {
      Serial.println("Invalid command. Available commands: AT, AT+RX, loopforever, read, clear, settime, getdate, updatetime");
    }

    SerialReadBuffer = "";
  }
}


void setTimeFromSerial() {
  Serial.println("Enter date and time in YYYY-MM-DD HH:MM:SS format:");
  String input = Serial.readStringUntil('\r');
  int year, month, day, hour, minute, second;
  if (sscanf(input.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
    DateTime newTime(year, month, day, hour, minute, second);
    rtc->adjust(newTime);
    Serial.println("RTC time has been set successfully.");
  } else {
    Serial.println("Invalid date and time format!");
  }
}

float getTemperature() {
  const float referenceResistor = 20000.0;  // Reference resistor (20kΩ)
  const float nominalResistance = 5000.0;   // Thermistor resistance at 25°C (5kΩ)
  const float nominalTemperature = 25.0;    // Nominal temperature (25°C)
  const float betaCoefficient = 3950.0;     // Beta coefficient (consult thermistor datasheet)
  
  // Add a delay for voltage stabilization (allow thermistor circuit to stabilize)
  delay(20);  // 200 ms delay to ensure voltage stabilizes after turning on the circuit

  // Read the ADC value from the thermistor pin using ADS1115
  int16_t adcReading = ads->readADC_SingleEnded(0);  // Read from AIN0

  // Convert the 16-bit ADC value to voltage (assuming GAIN_ONE is set, range is +-4.096V)
  float measuredVoltage = adcReading * (4.096 / 32768.0);  // ADS1115 is 16-bit ADC

  // Ensure the measured voltage is within the valid range
  if (measuredVoltage <= 0.0 || measuredVoltage >= 3.3) {
    Serial.println("Voltage out of range! Returning default value.");
    return 0.0;  // Return default value if voltage is out of range
  }

  // Calculate the thermistor resistance using the actual voltage
  float thermistorResistance = referenceResistor * (3.3 / measuredVoltage - 1);
  
  // Ensure the calculated resistance is positive and reasonable
  if (thermistorResistance <= 0.0) {
    Serial.println("Thermistor resistance out of range! Returning default value.");
    return 0.0;  // Return default value if resistance is invalid
  }

  // Use the Beta parameter equation to calculate the temperature in Kelvin
  float temperatureK = 1.0 / ((log(thermistorResistance / nominalResistance) / betaCoefficient) + (1.0 / (nominalTemperature + 273.15)));
  
  // Convert Kelvin to Celsius
  float temperatureC = temperatureK - 273.15;

  // Ensure temperature is within a reasonable range
  if (temperatureC < -40.0 || temperatureC > 125.0) {
    Serial.println("Temperature out of range! Returning default value.");
    return 0.0;  // Return default value if out of range
  }

  return temperatureC;
}

size_t writePosition = 0;                      // Track the current write position
const size_t MAX_FILE_SIZE = 2 * 1024 * 1024;  // 2MB
//const size_t MAX_FILE_SIZE = 420; // File size limit for testing

bool writeToFile(float temperature, unsigned long timestamp, const String& ackStatus) {
  // Open the file in append mode ("a"), which adds data to the end of the file
  File dataFile = LittleFS.open(DATA_FILE, "a");
  if (!dataFile) {
    Serial.println("Failed to open data file for writing");
    return false;
  }

  // Prepare the data to write
  String data = String(temperature, 2) + "," + String(timestamp) + "," + ackStatus + "\n";
  size_t dataLength = data.length();

  // Write data to the file
  size_t bytesWritten = dataFile.print(data);
  if (bytesWritten != dataLength) {
    Serial.println("Failed to write all data to the file.");
    dataFile.close();
    return false;
  }

  // Close the file
  dataFile.close();
  //Serial.println("Data written successfully.");
  return true;
}

void initializeWritePosition() {
  // Try to open the file to calculate the current write position
  File dataFile = LittleFS.open(DATA_FILE, "r");
  if (dataFile) {
    size_t fileSize = dataFile.size();
    if (fileSize < MAX_FILE_SIZE) {
      writePosition = fileSize;  // Start writing at the end of the file
    } else {
      writePosition = 0;  // Start at the beginning if the file is full
    }
    dataFile.close();
  } else {
    writePosition = 0;  // Start from the beginning if the file does not exist
  }
}

void readFromFile() {
  File dataFile = LittleFS.open(DATA_FILE, "r");
  if (!dataFile) {
    Serial.println("Failed to open data file for reading");
    return;
  }

  Serial.println("Reading data from file:");
  while (dataFile.available()) {
    String data = dataFile.readStringUntil('\n');
    int firstCommaIndex = data.indexOf(',');
    int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);

    if (firstCommaIndex != -1 && secondCommaIndex != -1) {
      float temperature = data.substring(0, firstCommaIndex).toFloat();
      unsigned long timestamp = data.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
      String ackStatus = data.substring(secondCommaIndex + 1);
      ackStatus.trim();

      DateTime dt = DateTime(timestamp);
      Serial.print("Temperature: ");
      Serial.print(temperature, 2);
      Serial.print(" °C, Timestamp: ");
      printDateTime(dt);
      Serial.print(", ACK Status: ");
      Serial.println(ackStatus);
    } else {
      Serial.println("Invalid data format");
    }
  }
  dataFile.close();
}

void clearFile() {
  LittleFS.remove(DATA_FILE);
  File dataFile = LittleFS.open(DATA_FILE, "w");
  if (!dataFile) {
    Serial.println("Failed to create new data file");
  } else {
    dataFile.close();
  }
}

String sanitizeData(const String& data) {
  String sanitizedData;
  for (char c : data) {
    if (!isspace(c)) {
      sanitizedData += c;
    }
  }
  return sanitizedData;
}

void resendLoggedData() {
  File dataFile = LittleFS.open(DATA_FILE, "r");
  if (!dataFile) {
    Serial.println("Failed to open data file for reading");
    return;
  }

  const int maxRetries = 3;            // Maximum retries for each entry
  const int maxEntriesToProcess = 20;  // Number of entries to process in one batch // amount of backfill packets to send
  int processedEntries = 0;

  while (dataFile.available() && processedEntries < maxEntriesToProcess) {
    yield();  // Prevent WDT reset

    String data = dataFile.readStringUntil('\n');
    if (data.length() > 0) {
      int firstCommaIndex = data.indexOf(',');
      int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);

      if (firstCommaIndex == -1 || secondCommaIndex == -1) {
        Serial.println("Invalid data format. Skipping entry.");
        continue;
      }

      float temperature = data.substring(0, firstCommaIndex).toFloat();
      unsigned long timestampData = data.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
      String ackStatusValue = data.substring(secondCommaIndex + 1);
      ackStatusValue.trim();

      if (ackStatusValue == "n") {
        DateTime timestamp(timestampData);

        // Validate timestamp
        if (timestamp.year() < 2020 || timestamp.year() > 2100) {
          Serial.println("Invalid timestamp. Skipping entry.");
          continue;
        }

        String timestampString = formatTimestamp(timestamp);

        String loRaData = "T:" + String(temperature, 2) + ",S:" + serialNumber + ",D:" + timestampString + " BackFill";

        Serial.print("LoRa Data (Transmitter): ");
        Serial.println(loRaData);

        uint32_t crcValue = crc32((const uint8_t*)loRaData.c_str(), loRaData.length());
        String crcString = String(crcValue, HEX);

        loRaData = crcString + "," + loRaData;

        if (validateData(loRaData)) {
          for (int attempt = 0; attempt < maxRetries; attempt++) {
            Serial.print("Retransmitting Data: " + loRaData);

            LoRa.beginPacket();
            LoRa.print(loRaData);
            LoRa.endPacket();
            Serial.print("\n");

            if (waitForACK()) {
              Serial.println("ACK received for retransmitted data.");
              ackStatusValue = "y";
              updateACKStatusInFile(dataFile, temperature, timestampData);
              break;
            }

            delay(50);  // Small delay between attempts
          }
        }
        processedEntries++;
        delay(50);  // Small delay between sending packets (was 500, can take long time to get through backfills)
      }
    }
  }
  dataFile.close();
  ackReceivedRecently = false;  // Reset the flag after resending data
}

String formatTimestamp(const DateTime& timestamp) {
  String timestampString = String(timestamp.year()) + "/" + 
                           (timestamp.month() < 10 ? "0" : "") + String(timestamp.month()) + "/" + 
                           (timestamp.day() < 10 ? "0" : "") + String(timestamp.day()) + " " + 
                           (timestamp.hour() < 10 ? "0" : "") + String(timestamp.hour()) + ":" + 
                           (timestamp.minute() < 10 ? "0" : "") + String(timestamp.minute()) + ":" + 
                           (timestamp.second() < 10 ? "0" : "") + String(timestamp.second());
  return timestampString;
}


bool validateData(const String& data) {
  for (char c : data) {
    if (!isPrintable(c)) {
      Serial.println("Data contains non-printable characters. Skipping entry.");
      return false;
    }
  }
  return true;
}

bool waitForACK() {
  unsigned long ackWaitStartTime = millis();
  while (millis() - ackWaitStartTime < 300) {
    yield();  // Prevent WDT reset

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String loRaResponse = "";
      while (LoRa.available()) {
        loRaResponse += (char)LoRa.read();
      }
      Serial.println("LoRa Response: " + loRaResponse);

      String trimmedLoRaResponse = cleanIncomingData(loRaResponse);

      if (isValidMessageFormat(trimmedLoRaResponse) && trimmedLoRaResponse.endsWith("ACK")) {
        return true;
      }
    }
  }
  return false;
}

void updateACKStatusInFile(File& dataFile, float temperature, unsigned long timestampData) {
  String newData = String(temperature, 2) + "," + String(timestampData) + ",y\n";
  File newDataFile = LittleFS.open(DATA_FILE, "r+");
  if (newDataFile) {
    newDataFile.seek(dataFile.position() - newData.length() - 1, SeekSet);
    newDataFile.print(newData);
    newDataFile.flush();
    newDataFile.close();
  } else {
    Serial.println("Failed to open data file for updating ACK status.");
  }
}

String cleanIncomingData(const String& rawData) {
  int startIndex = 0;
  int endIndex = rawData.length() - 1;

  while (startIndex < rawData.length() && isspace(rawData[startIndex])) {
    startIndex++;
  }

  while (endIndex >= 0 && isspace(rawData[endIndex])) {
    endIndex--;
  }

  String cleanedData = rawData.substring(startIndex, endIndex + 1);
  return cleanedData;
}


void sendTimeSyncCommand() {
  DateTime now = rtc->now();  // Get current time from RTC
  Serial.println(formatTimestamp(now));
  if (now.year() < 2000) {  // Basic validity check to ensure we have a valid time
    Serial.println("RTC time is invalid, skipping time sync.");
    return;  // Skip sending time sync if the time is invalid
  }

  String timeSyncMessage = String(serialNumber) + "," + formatTimestamp(now);  // Prepare time sync message
  Serial.println("Sending Time Sync Command: " + timeSyncMessage);
  
  LoRa.beginPacket();
  LoRa.print(timeSyncMessage);
  LoRa.endPacket();
  Serial.println("Time sync command sent.");
}
// Read the last time sync from LittleFS
unsigned long readLastTimeSync() {
  File file = LittleFS.open(LAST_TIME_SYNC_FILE, "r");
  if (!file) {
    Serial.println("No last time sync file found.");
    return 0;  // No sync has been performed before
  }

  String lastTime = file.readStringUntil('\n');
  file.close();

  unsigned long lastSyncTime = lastTime.toInt();  // Convert the string to an unsigned long
  if (lastSyncTime == 0) {
    Serial.println("Invalid last time sync data.");
  }

  return lastSyncTime;  // Return the Unix timestamp
}

// Write the last time sync to LittleFS
void writeLastTimeSync(unsigned long currentTime) {
    // Ensure LittleFS is mounted
   

    File file = LittleFS.open(LAST_TIME_SYNC_FILE, "w");
    if (!file) {
        Serial.println("Failed to write last time sync file.");
        return;
    }

    file.println(String(currentTime));  // Save the current time as Unix timestamp
    file.close();
    Serial.println("Last time sync written to LittleFS: " + String(currentTime));
      // Unmount LittleFS after operation
}

// Process the response from the base unit and update RTC
void processTimeSyncResponse(const String& message) {
    Serial.println("Processing time sync response: " + message);

    if (message.startsWith(String(serialNumber) + ",")) {
        Serial.println("Serial number matches.");

        int firstCommaIndex = message.indexOf(",");
        int secondCommaIndex = message.indexOf(",", firstCommaIndex + 1);

        if (firstCommaIndex != -1 && secondCommaIndex != -1) {
            String datetimeStr = message.substring(firstCommaIndex + 1, secondCommaIndex);  // Extract datetime string
            String updateFlag = message.substring(secondCommaIndex + 1);                    // Extract the "UPDATE" flag

            Serial.println("Extracted datetime: " + datetimeStr);
            Serial.println("Extracted flag: " + updateFlag);

            if (updateFlag == "UPDATE") {
                Serial.println("UPDATE flag detected. Processing time update...");

                // Parse the datetime string and set the RTC in the same way as setTimeFromSerial
                DateTime newDateTime = parseDateTime(datetimeStr);

                if (newDateTime.year() > 2000) {  // Basic validity check
                    Serial.println("Valid datetime received. Adjusting RTC...");
                    rtc->adjust(newDateTime);  // Adjust RTC to the parsed datetime
                    Serial.println("RTC updated with new datetime: ");
                    printDateTime(newDateTime);  // Print the updated datetime

                    // Save the valid datetime to LittleFS once
                    unsigned long currentUnixTime = newDateTime.unixtime();
                    saveTimeToLittleFS(newDateTime);   // Save valid time
                    writeLastTimeSync(currentUnixTime);  // Save last time sync only once
                    Serial.println("Saved valid datetime to LittleFS: " + String(currentUnixTime));
                } else {
                    Serial.println("Invalid datetime received.");
                }
            } else {
                Serial.println("No UPDATE flag detected.");
            }
        } else {
            Serial.println("Invalid message format.");
        }
    } else {
        Serial.println("Serial number mismatch.");
    }
}

// Parse received datetime string
DateTime parseDateTime(const String& datetimeStr) {
  int year, month, day, hour, minute, second;

  // Parse the datetime string in the format "YYYY/MM/DD HH:MM:SS"
  int parsedItems = sscanf(datetimeStr.c_str(), "%d/%d/%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);

  if (parsedItems == 6) {
    Serial.println("Parsed datetime successfully.");
    return DateTime(year, month, day, hour, minute, second);
  } else {
    Serial.println("Failed to parse datetime. Using default value.");
    return DateTime();  // Return an invalid DateTime object if parsing fails
  }
}



bool isBST(const DateTime& dt) {
  // BST runs from the last Sunday of March to the last Sunday of October.
  int year = dt.year();

  // Get the last Sunday of March
  DateTime lastSundayOfMarch = DateTime(year, 3, (31 - (5 * year / 4 + 4) % 7));
  // Get the last Sunday of October
  DateTime lastSundayOfOctober = DateTime(year, 10, (31 - (5 * year / 4 + 1) % 7));

  return (dt >= lastSundayOfMarch && dt < lastSundayOfOctober);
}

// Function to read battery voltage from INA226
float readBatteryVoltageINA() {
  return ina226->getBusVoltage_mV() / 1000.0;  // Convert mV to V
}

void saveTimeToLittleFS(DateTime now) {
    // Ensure LittleFS is mounted
   

    File file = LittleFS.open("/lastValidTime.txt", "w");
    if (!file) {
        Serial.println("Failed to open file to save time.");
        return;
    }
    unsigned long currentTime = now.unixtime();
    file.println(String(currentTime));  // Save current time as Unix timestamp
    file.close();
    Serial.println("Current time saved to LittleFS: " + String(currentTime));
     // Unmount LittleFS after operation
}

unsigned long restoreTimeFromLittleFS() {
  File file = LittleFS.open("/lastValidTime.txt", "r");
  if (!file) {
    Serial.println("No saved time found in LittleFS.");
    return 0;  // No valid saved time
  }

  String savedTimeStr = file.readStringUntil('\n');
  file.close();
  unsigned long savedTime = savedTimeStr.toInt();  // Convert saved string to Unix time

  if (savedTime > 0 && savedTime < 2147483647) {  // Validate the range of Unix timestamp
    Serial.println("Restored time from LittleFS: " + String(savedTime));
  } else {
    Serial.println("Invalid time found in LittleFS.");
    savedTime = 0;
  }

  return savedTime;
}


void checkForTemperatureData() {
    // Ensure LittleFS is mounted
    

    // Open the file in read mode
    File dataFile = LittleFS.open(DATA_FILE, "r");
    if (!dataFile) {
        Serial.println("Temperature data file not found.");
        return;
    }

    // Check if the file contains any valid data
    if (dataFile.size() > 0) {
        Serial.println("Temperature data exists in LittleFS:");
        
        // Read the first line as a quick check
        String firstEntry = dataFile.readStringUntil('\n');
        Serial.println("First entry: " + firstEntry);

        // Optionally, parse and validate the data
        int firstCommaIndex = firstEntry.indexOf(',');
        int secondCommaIndex = firstEntry.indexOf(',', firstCommaIndex + 1);

        if (firstCommaIndex != -1 && secondCommaIndex != -1) {
            float temperature = firstEntry.substring(0, firstCommaIndex).toFloat();
            unsigned long timestamp = firstEntry.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
            String ackStatus = firstEntry.substring(secondCommaIndex + 1);
            ackStatus.trim();

            Serial.print("Parsed Temperature: ");
            Serial.println(temperature);
            Serial.print("Parsed Timestamp: ");
            Serial.println(timestamp);
            Serial.print("ACK Status: ");
            Serial.println(ackStatus);
        } else {
            Serial.println("Invalid format in temperature data file.");
        }
    } else {
        Serial.println("No temperature data found in LittleFS.");
    }

    dataFile.close();
}

void checkForBackfill() {
    File dataFile = LittleFS.open(DATA_FILE, "r");
    if (!dataFile) {
        Serial.println("Failed to open data file for reading");
        return;
    }

    bool hasUnacknowledgedData = false;

    while (dataFile.available()) {
        String line = dataFile.readStringUntil('\n');
        line.trim(); // Remove whitespace or newline characters

        // Parse the line to check for unacknowledged data
        int firstCommaIndex = line.indexOf(',');
        int secondCommaIndex = line.indexOf(',', firstCommaIndex + 1);

        if (firstCommaIndex != -1 && secondCommaIndex != -1) {
            String ackStatus = line.substring(secondCommaIndex + 1); // Extract the ACK status
            ackStatus.trim(); // Trim the ACK status separately
            if (ackStatus == "n") {
                hasUnacknowledgedData = true;
                break; // No need to check further if unacknowledged data is found
            }
        }
    }

    dataFile.close();

    if (hasUnacknowledgedData) {
        Serial.println("Pending unacknowledged backfill data found. Preparing to resend...");
        resendLoggedData();
    } else {
       // Serial.println("No unacknowledged backfill data found.");
    }
}

// Helper function to extract values from messages
String extractValue(const String &data, const String &key) {
  int startIndex = data.indexOf(key + ":");
  if (startIndex == -1) return "";
  startIndex += key.length() + 1;
  int endIndex = data.indexOf(",", startIndex);
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex);
}

// Helper function to connect to Wi-Fi
bool connectToWiFi(const String &ssid, const String &password) {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to Wi-Fi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 30000) { // Timeout after 30 seconds
            Serial.println("\nFailed to connect to Wi-Fi.");
            return false;
        }
    }
    Serial.println("\nConnected to Wi-Fi.");
    return true;
}


bool isNewFirmwareAvailable() {
    // Clean up sensor data in LittleFS
    Serial.println("Deleting all sensor data files from LittleFS...");
   
    Dir dir = LittleFS.openDir("/");
    while (dir.next()) {
        if (dir.fileName().startsWith("/sensor")) { // Delete files starting with 'sensor'
            if (LittleFS.remove(dir.fileName())) {
                Serial.println("Deleted file: " + dir.fileName());
            } else {
                Serial.println("Failed to delete file: " + dir.fileName());
            }
        }
    }
     // Unmount LittleFS

    // Log heap status before HTTP request
    Serial.printf("Heap Fragmentation: %d%%, Free Heap: %u bytes\n", ESP.getHeapFragmentation(), ESP.getFreeHeap());

    // Initialize retry parameters
    const int maxRetries = 3;
    const int retryDelay = 2000;
    int retryCount = 0;

    while (retryCount < maxRetries) {
        HTTPClient http;

        // Ensure WiFi is still connected before making a request
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected. Aborting firmware check.");
            return false;
        }

        Serial.println("Attempting to fetch firmware version...");
        String url = "https://raw.githubusercontent.com/thermalsystemsltd/sensorfirmware/refs/heads/main/version";
        url += "?nocache=" + String(millis()); // Avoid cached responses

        if (!http.begin(*espClientSecure, url)) {
            Serial.println("Failed to initialize HTTP client.");
            http.end(); // Clean up
            retryCount++;
            delay(retryDelay);
            continue;
        }

        int httpCode = http.GET();
        if (httpCode > 0) {
            Serial.printf("HTTP GET Request sent. Response code: %d\n", httpCode);
            if (httpCode == HTTP_CODE_OK) {
                String newVersion = http.getString();
                newVersion.trim(); // Remove extra whitespace or newlines
                Serial.println("Raw HTTP Response: " + newVersion);

                Serial.println("Available Firmware Version: " + newVersion);
                Serial.println("Current Firmware Version: " + String(FIRMWARE_VERSION));

                http.end(); // Clean up resources

                if (newVersion != String(FIRMWARE_VERSION)) {
                    Serial.println("New firmware version detected. Update required.");
                    return true;
                } else {
                    Serial.println("Firmware is up to date. No update required.");
                    return false;
                }
            } else {
                Serial.printf("HTTP response not OK. Code: %d\n", httpCode);
            }
        } else {
            Serial.printf("HTTP GET request failed. Error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end(); // Clean up resources
        retryCount++;
        delay(retryDelay);
    }

    Serial.println("Failed to fetch firmware version after maximum retries.");
    return false;
}






// Helper function to perform OTA update
bool performOTAUpdate(const String& firmwareURL, const String& firmwareBinURL) {
    Serial.println("Starting OTA update...");
    
    // Save the current serial number to LittleFS
    saveSerialNumberToLittleFS(serialNumber);
    
    Serial.printf("Fetching firmware from: %s\n", firmwareBinURL.c_str());

    ESPhttpUpdate.rebootOnUpdate(false); // Manually handle reboot

    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW); // Indicate update progress using LED

    // Monitor progress
    ESPhttpUpdate.onProgress([](int current, int total) {
        Serial.printf("OTA Progress: %d/%d bytes (%d%%)\r", current, total, (current * 100) / total);
    });

    t_httpUpdate_return ret = ESPhttpUpdate.update(*espClientSecure, firmwareBinURL);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("OTA Update Failed. Error: (%d) %s\n",
                          ESPhttpUpdate.getLastError(),
                          ESPhttpUpdate.getLastErrorString().c_str());
            return false;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("No OTA updates available.");
            return false;
        case HTTP_UPDATE_OK:
            Serial.println("\nOTA Update Successful. Ready to reboot.");
            return true;
    }
    return false; // Should not be reached
}



/*
void checkForFirmwareUpdate() {
    if (LoRa.parsePacket()) {
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        Serial.println("LoRa Message Received: " + message);

        if (message.startsWith("UPDATE:START")) {
            String sensorID = extractValue(message, "SENSOR");
            String ssid = extractValue(message, "SSID");
            String password = extractValue(message, "PASS");
            String firmwareBinURL = extractValue(message, "URL");

            if (sensorID == serialNumber) {
                Serial.println("Firmware update command received for this sensor.");

                if (connectToWiFi(ssid, password)) {
                    if (!firmwareBinURL.isEmpty()) {
                        performOTAUpdate(sensor_firmware_version_url, firmwareBinURL);
                    } else {
                        Serial.println("Invalid firmware URL. Update aborted.");
                    }
                    WiFi.disconnect();
                } else {
                    Serial.println("Failed to connect to Wi-Fi for OTA.");
                }
            } else {
                Serial.println("Firmware update ignored: Sensor ID mismatch.");
            }
        }
    }
}
*/


void testBinaryUrl() {
    HTTPClient http;

    Serial.printf("Testing connection to %s\n", firmwareBinURL);
    if (!http.begin(*espClientSecure, firmwareBinURL)) {
        Serial.println("Failed to initialize HTTP client.");
        return;
    }

    int httpCode = http.GET();
    if (httpCode > 0) {
        Serial.printf("HTTP GET request sent. Response code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK) {
            Serial.println("Binary URL is accessible and ready for download.");
            String payload = http.getString();
            Serial.println("Payload:");
            Serial.println(payload); // Print the response payload
        }
    } else {
        Serial.printf("HTTP GET request failed. Error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
}
////////////////////////
// Function to decode Base64-encoded SSID and password
bool decodeUpdate(const String& encodedSSID, const String& encodedPassword, String& decodedSSID, String& decodedPassword) {
    // Buffers to hold the input data for decoding
    unsigned char ssidInput[128] = {0};
    unsigned char passwordInput[128] = {0};

    // Copy encoded data into non-const buffers
    strncpy((char*)ssidInput, encodedSSID.c_str(), sizeof(ssidInput) - 1);
    strncpy((char*)passwordInput, encodedPassword.c_str(), sizeof(passwordInput) - 1);

    // Buffers to store the decoded data
    unsigned char ssidBuffer[128] = {0};
    unsigned char passwordBuffer[128] = {0};

    // Decode SSID
    unsigned int ssidLength = decode_base64(ssidInput, ssidBuffer);
    ssidBuffer[ssidLength] = '\0'; // Null-terminate the decoded SSID

    // Decode Password
    unsigned int passwordLength = decode_base64(passwordInput, passwordBuffer);
    passwordBuffer[passwordLength] = '\0'; // Null-terminate the decoded Password

    // Assign the decoded values to output Strings
    decodedSSID = String((char*)ssidBuffer);
    decodedPassword = String((char*)passwordBuffer);

    // Debugging output
    Serial.print("Decoded SSID: ");
    Serial.println(decodedSSID);
    Serial.print("Decoded Password: ");
    Serial.println(decodedPassword);

    // Return true if decoding was successful
    return (ssidLength > 0 && passwordLength > 0);
}

void saveSerialNumberToLittleFS(const String &serial) {
    

    File file = LittleFS.open("/serialNumber.txt", "w");
    if (!file) {
        Serial.println("Failed to create file for serial number.");
        return;
    }

    file.println(serial);  // Write the serial number
    file.close();
    Serial.println("Serial number saved to LittleFS: " + serial);
}

String readSerialNumberFromLittleFS() {
    

    File file = LittleFS.open("/serialNumber.txt", "r");
    if (!file) {
        //Serial.println("Serial number file not found.");
        return "";
    }

    String serial = file.readStringUntil('\n');  // Read the serial number
    serial.trim();  // Remove any extra whitespace
    file.close();

    if (serial.isEmpty()) {
        Serial.println("No serial number found in file.");
        return "";
    }

    Serial.println("Serial number read from LittleFS: " + serial);
    return serial;
}

bool isChannelClear() {
  LoRa.idle();  // Ensure LoRa module is not actively transmitting
  int rssi = LoRa.rssi();  // Get the current RSSI value
  //Serial.print("Current RSSI: ");
  //Serial.println(rssi);
  return rssi < LBT_RSSI_THRESHOLD;
}

///////////////////////RTC ALARM CODE ////////////////

void handleFirmwareUpdate(const String& message) {
    Serial.println("Firmware update command received. Processing...");

    // Extract values from the LoRa response
    String sensorID = extractValue(message, "SENSOR");
    String encodedSSID = extractValue(message, "SSID");
    String encodedPassword = extractValue(message, "PASS");

    // Validate that this update is intended for the current sensor
    if (sensorID == serialNumber) {
        Serial.println("Firmware update is for this sensor. Initiating update...");

        // Variables to store the decoded SSID and password
        String decodedSSID;
        String decodedPassword;

        // Decode the SSID and password
        if (decodeUpdate(encodedSSID, encodedPassword, decodedSSID, decodedPassword)) {
            // Attempt to connect to Wi-Fi
            if (connectToWiFi(decodedSSID, decodedPassword)) {
                Serial.println("Connected to Wi-Fi. Checking for firmware update...");
                if (isNewFirmwareAvailable()) {
                    if (performOTAUpdate(sensor_firmware_version_url, firmwareBinURL)) {
                        Serial.println("Update successful. Signaling timer to power cycle the device.");
                        Serial.flush();
                        WiFi.disconnect(true); // Disconnect and turn off radio
                        delay(100);

                        pinMode(10, OUTPUT);
                        digitalWrite(10, HIGH); // Signal TPL5110 to cut power

                        // The device will be powered down by the timer.
                        // Loop indefinitely to prevent further execution.
                        while (true) {
                            delay(1000);
                        }
                    }
                }
                WiFi.disconnect(true); // Disconnect if no update or if it failed
            } else {
                Serial.println("Failed to connect to Wi-Fi for firmware update.");
            }
        } else {
            Serial.println("Failed to decode SSID or Password.");
        }
    } else {
        Serial.println("Firmware update command ignored: Sensor ID mismatch.");
    }
}
