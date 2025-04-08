#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t MqttTaskHandle = NULL;

// Cảm biến MQ-2
#define MQ2PIN 34 // Chân Analog kết nối với chân AO của MQ-2 (có thể thay đổi)
#define MQ2_THRESHOLD 500 // Ngưỡng để phát hiện khí (có thể điều chỉnh)

// Biến temperature và humidity (gán giá trị mặc định)
float temperature = 25.0; // Giá trị mặc định (có thể thay đổi)
float humidity = 60.0;    // Giá trị mặc định (có thể thay đổi)

// WiFi Credentials (Thay bằng WiFi của bạn)
const char* ssid = "RedmiK60Pro";      
const char* password = "taokocon123456";

// MQTT Server CoreIoT
const char* mqtt_server = "app.coreiot.io";  
const int mqtt_port = 1883;
const char* access_token = "ECKmVyxHPt9N9Cr5Wn7j";
const char* mqtt_topic = "v1/devices/me/telemetry";

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Kết nối WiFi
void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
        Serial.print("ESP32 IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi! Please check credentials or signal.");
    }
}

// Kiểm tra DNS để phân giải "app.coreiot.io"
void testDNS() {
    IPAddress mqttServerIP;
    if (WiFi.hostByName(mqtt_server, mqttServerIP)) {
        Serial.print("Resolved IP for MQTT Broker: ");
        Serial.println(mqttServerIP);
    } else {
        Serial.println("Failed to resolve MQTT Broker IP");
    }
}

// Kết nối MQTT bằng Access Token
void connectMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client", access_token, NULL)) {
            Serial.println("Connected to MQTT!");
        } else {
            Serial.print("Failed, rc=");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

// Task 1: In ra console
void Task1(void *pvParameters) {
    Serial.println("Task1 started!");
    while (1) {
        Serial.println("Hello from Task1");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task 2: In ra console
void Task2(void *pvParameters) {
    Serial.println("Task2 started!");
    while (1) {
        Serial.println("Hello from Task2");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// Task 3: Đọc dữ liệu từ MQ-2
int mq2Value = 0; // Giá trị nồng độ khí từ MQ-2
bool gasDetected = false; // Trạng thái phát hiện khí

void Task3(void *pvParameters) {
    Serial.println("Task3 started!");
    while (1) {
        // Đọc giá trị analog từ cảm biến MQ-2
        mq2Value = analogRead(MQ2PIN);

        // Kiểm tra nếu nồng độ khí vượt ngưỡng
        if (mq2Value > MQ2_THRESHOLD) {
            gasDetected = true;
            Serial.println("Phát hiện khí nguy hiểm!");
        } else {
            gasDetected = false;
            Serial.println("Không phát hiện khí nguy hiểm.");
        }

        // In giá trị nồng độ khí
        Serial.print("Nồng độ khí (MQ-2): ");
        Serial.println(mq2Value);

        vTaskDelay(pdMS_TO_TICKS(2000)); // Đọc mỗi 2 giây
    }
}

// Task 4: Gửi dữ liệu lên CoreIoT bằng MQTT
void MqttTask(void *pvParameters) {
    Serial.println("MqttTask started!");
    while (1) {
        if (!client.connected()) {
            connectMQTT();
        }
        client.loop();

        // Gửi dữ liệu từ MQ-2 (bao gồm temperature, humidity, gasValue, gasDetected) lên CoreIoT
        char payload[150];
        snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f, \"gasValue\": %d, \"gasDetected\": %d}", 
                 temperature, humidity, mq2Value, gasDetected);
        client.publish(mqtt_topic, payload);

        Serial.print("Sent to MQTT: ");
        Serial.println(payload);

        vTaskDelay(pdMS_TO_TICKS(5000)); // Gửi dữ liệu mỗi 5 giây
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Delay để Serial Monitor ổn định
    Serial.println("Program started!");

    // Kết nối WiFi
    connectWiFi();

    // Kiểm tra DNS của server
    testDNS();

    // Khởi tạo chân cảm biến MQ-2
    pinMode(MQ2PIN, INPUT);
    Serial.println("Cảm biến MQ-2 đã được khởi tạo.");

    // Kiểm tra giá trị ban đầu từ MQ-2
    int initialValue = analogRead(MQ2PIN);
    Serial.print("Giá trị ban đầu từ MQ-2: ");
    Serial.println(initialValue);

    // Delay để MQ-2 làm nóng
    Serial.println("Đang làm nóng MQ-2, vui lòng chờ 5s ...");
    delay(5000); // Delay 2 phút để MQ-2 làm nóng
    Serial.println("MQ-2 đã sẵn sàng!");

    // Tạo Tasks với FreeRTOS
    xTaskCreate(Task1, "Task1", 2048, NULL, 1, &Task1Handle);
    xTaskCreate(Task2, "Task2", 2048, NULL, 1, &Task2Handle);
    xTaskCreate(Task3, "MQ2Task", 4096, NULL, 1, &Task3Handle);
    xTaskCreate(MqttTask, "MQTTTask", 8192, NULL, 1, &MqttTaskHandle);
    Serial.println("Tasks created!");
}

void loop() {
    // FreeRTOS quản lý Tasks
}