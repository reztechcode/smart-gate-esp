#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Preferences.h>

/* --- PIN CONFIG --- */
#define RFID_SS 17
#define RFID_RST 21
#define RFID_SCK 5
#define RFID_MOSI 18
#define RFID_MISO 19
#define LCD_SDA 22
#define LCD_SCL 23
#define PIR_PIN 27
#define RELAY_PIN 16
#define SERVO_PIN 4
#define BUZZER_PIN 26

#define MQTT_HOST "broker.rezweb.my.id"
#define MQTT_PORT 0000
#define MQTT_USER "#"
#define MQTT_PASS "#"

#define TOPIC_COMMAND "iot/gate/command"
#define TOPIC_CONFIG "iot/gate/config"
#define TOPIC_STATUS "iot/gate/status"
#define TOPIC_ACCESS "iot/gate/access"
#define TOPIC_MODE "iot/gate/mode"

WiFiClient espClient;
PubSubClient mqtt(espClient);
MFRC522 rfid(RFID_SS, RFID_RST);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo gateServo;
Preferences preferences;

enum Mode
{
    MODE_NORMAL,
    MODE_GATE_OPEN,
    MODE_GATE_CLOSING,
    MODE_REGISTER
};
Mode currentMode = MODE_NORMAL;

bool isOffline = true;
int autoCloseSec = 15;
int sensorGraceSec = 5;
String localMasterCards = "";
unsigned long gateOpenedAt = 0;
unsigned long lastSensorActive = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastMqttRetry = 0;
int currentServoPos = 180;
bool gracePhaseStarted = false;

unsigned long registerTimeoutAt = 0;

void updateLcdStatus(String line1, String line2 = "");
void showStandby();
void publishData(String eventName);
bool anySensorActive();
void openGate();
void closeGate();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void handleConnection();
void handleRfidScan();
void handleGateLogic();

void setup()
{
    Serial.begin(115200);
    pinMode(PIR_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    Wire.begin(LCD_SDA, LCD_SCL);
    lcd.init();
    lcd.backlight();
    updateLcdStatus("GATE SYSTEM", "INITIALIZING...");

    preferences.begin("gate_auth", true);
    localMasterCards = preferences.getString("master_cards", "");
    autoCloseSec = preferences.getInt("auto_close", autoCloseSec);
    sensorGraceSec = preferences.getInt("grace_sec", sensorGraceSec);
    preferences.end();

    gateServo.attach(SERVO_PIN);
    gateServo.write(180);

    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
    rfid.PCD_Init();
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);

    WiFiManager wm;
    wm.setConfigPortalTimeout(60);

    // Callback saat masuk AP Mode
    wm.setAPCallback([](WiFiManager *wm)
                     {
    IPAddress ip = WiFi.softAPIP();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SETUP WIFI AP");
    lcd.setCursor(0,1);
    lcd.print(ip.toString()); });

    if (!wm.autoConnect("GATE-SECURE-AP"))
    {
        isOffline = true;
    }

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    showStandby();
}

void loop()
{
    handleConnection();
    handleRfidScan();
    handleGateLogic();
    yield();
}

void publishData(String eventName)
{
    if (!mqtt.connected())
        return;
    JsonDocument doc;
    doc["event"] = eventName;
    doc["rssi"] = WiFi.RSSI();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["wifi_ssid"] = WiFi.SSID();
    doc["mac_address"] = WiFi.macAddress();
    doc["chip_model"] = ESP.getChipModel();
    doc["chip_revision"] = ESP.getChipRevision();
    doc["chip_cores"] = ESP.getChipCores();

    doc["uptime"] = millis() / 1000;
    doc["mode"] = (currentMode == MODE_REGISTER) ? "REGISTER" : "NORMAL";
    String out;
    serializeJson(doc, out);
    mqtt.publish(TOPIC_STATUS, out.c_str());
}

void handleConnection()
{
    unsigned long now = millis();
    if (WiFi.status() != WL_CONNECTED)
    {
        isOffline = true;
        return;
    }

    if (!mqtt.connected())
    {
        isOffline = true;
        if (now - lastMqttRetry > 5000)
        {
            lastMqttRetry = now;
            if (mqtt.connect("ESP32-GATE", MQTT_USER, MQTT_PASS))
            {
                mqtt.subscribe(TOPIC_COMMAND);
                mqtt.subscribe(TOPIC_CONFIG);
                mqtt.subscribe(TOPIC_MODE);
                isOffline = false;
                publishData("online");
                showStandby();
            }
        }
    }
    else
    {
        isOffline = false;
        mqtt.loop();
        if (now - lastHeartbeat > 30000)
        {
            lastHeartbeat = now;
            publishData("heartbeat");
        }
    }
}

void handleGateLogic()
{
    unsigned long now = millis();

    if (currentMode == MODE_REGISTER)
    {
        if (now > registerTimeoutAt)
        {
            currentMode = MODE_NORMAL;
            updateLcdStatus("REGISTER SELESAI", "KEMBALI NORMAL");
            delay(2000);
            showStandby();
        }
        else if (now - lastLcdUpdate > 500)
        {
            int rem = (registerTimeoutAt - now) / 1000;
            updateLcdStatus("MODE REGISTER", "TIMEOUT: " + String(rem) + "s");
            lastLcdUpdate = now;
        }
        return;
    }

    if (currentMode != MODE_GATE_OPEN)
        return;

    bool active = anySensorActive();
    if (active)
        lastSensorActive = now;

    int timeLeft = (autoCloseSec * 1000 - (long)(now - gateOpenedAt) + 999) / 1000;
    if (timeLeft < 0)
        timeLeft = 0;

    int graceLeft = (sensorGraceSec * 1000 - (long)(now - lastSensorActive) + 999) / 1000;
    if (graceLeft < 0)
        graceLeft = 0;

    if (timeLeft <= 0 && !gracePhaseStarted && !active)
    {
        lcd.clear();
        gracePhaseStarted = true;
    }

    if (now - lastLcdUpdate > 250)
    {
        lcd.setCursor(0, 0);
        lcd.print("Buka: ");
        lcd.print(autoCloseSec);
        lcd.print("s   ");
        lcd.setCursor(0, 1);
        if (active)
        {
            lcd.print("> ADA OBJEK <  ");
            gracePhaseStarted = false;
        }
        else
        {
            if (timeLeft > 0)
            {
                lcd.print("Tutup: ");
                lcd.print(timeLeft);
                lcd.print("s  ");
            }
            else
            {
                lcd.print("Cek Area: ");
                lcd.print(graceLeft);
                lcd.print("s   ");
            }
        }
        lastLcdUpdate = now;
    }

    if (timeLeft <= 0 && !active && graceLeft <= 0)
    {
        delay(500);
        closeGate();
        gracePhaseStarted = false;
    }
}

void openGate()
{
    currentMode = MODE_GATE_OPEN;
    gracePhaseStarted = false;
    updateLcdStatus("AKSES DITERIMA", "MEMBUKA GERBANG");
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
    for (int pos = currentServoPos; pos >= 0; pos--)
    {
        gateServo.write(pos);
        currentServoPos = pos;
        delay(15);
    }
    delay(500);
    digitalWrite(RELAY_PIN, LOW);
    lcd.clear();
    gateOpenedAt = millis();
    lastSensorActive = millis();
    publishData("gate_opened");
}

void closeGate()
{
    currentMode = MODE_GATE_CLOSING;
    updateLcdStatus("PROSES TUTUP", "MOHON TUNGGU");
    
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);

    for (int pos = currentServoPos; pos <= 180; pos++)
    {
        if (anySensorActive())
        {
            updateLcdStatus("ADA OBJEK!", "STOP & TUNGGU");
            
            digitalWrite(RELAY_PIN, LOW);
            
            digitalWrite(BUZZER_PIN, HIGH); 

            while (anySensorActive())
            {
                delay(100);
            }

            updateLcdStatus("AMAN", "LANJUT TUTUP");
            digitalWrite(BUZZER_PIN, LOW); 
            
            digitalWrite(RELAY_PIN, HIGH); 
            delay(500);
        }

        gateServo.write(pos);
        currentServoPos = pos;

        if (pos % 15 == 0) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(20); 
            digitalWrite(BUZZER_PIN, LOW);
        }
        delay(25);
    }

    digitalWrite(RELAY_PIN, LOW);
    currentMode = MODE_NORMAL;
    publishData("gate_closed");
    
    for(int i=0; i<2; i++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(100);
        digitalWrite(BUZZER_PIN, LOW); delay(50);
    }

    updateLcdStatus("GERBANG", "TERTUTUP");
    delay(1500);
    lcd.clear();
    showStandby();
}

void handleRfidScan()
{
    if (currentMode != MODE_NORMAL && currentMode != MODE_REGISTER)
        return;
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
        return;

    String uid = "";
    for (byte i = 0; i < rfid.uid.size; i++)
    {
        uid += (rfid.uid.uidByte[i] < 0x10 ? "0" : "") + String(rfid.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();

    if (currentMode == MODE_REGISTER)
    {
        updateLcdStatus("KARTU TERDETEKSI", "MENGIRIM KE SYS");
        JsonDocument reg;
        reg["uid"] = uid;
        reg["method"] = "rfid";
        reg["action"] = "register_attempt";
        String out;
        serializeJson(reg, out);
        mqtt.publish(TOPIC_ACCESS, out.c_str());
        delay(1000);
    }
    else
    {
        if (isOffline)
        {
            if (localMasterCards.indexOf(uid) >= 0)
                openGate();
            else
            {
                updateLcdStatus("DITOLAK (OFF)", "ID:" + uid.substring(0, 8));
                delay(2000);
                showStandby();
            }
        }
        else
        {
            JsonDocument acc;
            acc["uid"] = uid;
            acc["method"] = "rfid";
            String out;
            serializeJson(acc, out);
            mqtt.publish(TOPIC_ACCESS, out.c_str());
            updateLcdStatus("VERIFIKASI...", "HARAP TUNGGU");
        }
    }
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    JsonDocument doc;
    deserializeJson(doc, payload, length);
    String t = String(topic);

    if (t == TOPIC_MODE)
    {
        String modeReq = doc["mode"] | "normal";
        if (modeReq == "register")
        {
            currentMode = MODE_REGISTER;
            int timeout = doc["timeout"] | 60;
            registerTimeoutAt = millis() + (timeout * 1000);
            updateLcdStatus("MODE REGISTER", "TAP KARTU BARU");
        }
        else if (modeReq == "normal")
        {
            if (doc["status"] == "success")
            {
                updateLcdStatus("REGISTRASI", "BERHASIL!");
                delay(2000);
            }
            currentMode = MODE_NORMAL;
            showStandby();
        }
        
    }
    else if (t == TOPIC_COMMAND)
    {
        String action = doc["action"] | "";
        if (action == "open")
            openGate();
        else if (action == "deny")
        {
            updateLcdStatus("AKSES DITOLAK", doc["reason"] | "Dilarang");
            delay(2000);
            showStandby();
        }
        else if (action == "ping") publishData("pong");
    }
    else if (t == TOPIC_CONFIG)
    {
        JsonObject g = doc["gerbang_utama"];
        if (!g.isNull())
        {
            lcd.clear();
            updateLcdStatus("SYNC PROGRESS", "LOAD CONFIG");
            autoCloseSec = g["gate_timing"]["auto_close_delay_sec"] | autoCloseSec;
            sensorGraceSec = g["gate_timing"]["sensor_grace_sec"] | sensorGraceSec;
            JsonArray arr = g["allowed_cards"];
            if (!arr.isNull())
            {
                localMasterCards = "";
                for (JsonVariant v : arr)
                {
                    localMasterCards += v.as<String>() + ",";
                }
                preferences.begin("gate_auth", false);
                preferences.putString("master_cards", localMasterCards);
                preferences.putInt("auto_close", autoCloseSec);
                preferences.putInt("grace_sec", sensorGraceSec);
                preferences.end();
                delay(1000);
            }
        }
        updateLcdStatus("SYNC BERHASIL", "DATA DISIMPAN");
        delay(1500);
        showStandby();
    }
}

bool anySensorActive() {
    bool isDetected = (digitalRead(PIR_PIN) == HIGH);
    if (isDetected && (currentMode == MODE_GATE_CLOSING || currentMode == MODE_GATE_OPEN)) {
        digitalWrite(BUZZER_PIN, HIGH); 
    } else if (currentMode != MODE_GATE_CLOSING) {  
        digitalWrite(BUZZER_PIN, LOW);
    }
    
    return isDetected;
}
void updateLcdStatus(String line1, String line2)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void showStandby()
{
    if (isOffline)
        updateLcdStatus("MODE: OFFLINE", "SCAN KARTU");
    else
        updateLcdStatus("MODE: ONLINE", "READY SCAN");
}
