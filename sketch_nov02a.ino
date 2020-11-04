#include "RotaryEncoder.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define LED_PIN   2 //D4
#define MOTOR_A_PIN 13 //D7
#define MOTOR_B_PIN 4 //D2
#define MOTOR_EN_PIN 16 //D0

#define ENCODER_A_PIN 12 //D6
#define ENCODER_B_PIN 14 //D5

#define MOTOR_HOME_RETRACT_COUNT 180
#define MOTOR_HOME_STALL_THRESHOLD 50
#define MOTOR_STEPS_IN_ONE_TURN 1364

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define MQTT_SERVER "192.168.118.10"
#define MQTT_USER "guest"
#define MQTT_PASSWORD "guest"
#define MQTT_TOPIC_STATUS "lock.status"
#define MQTT_TOPIC_COMMAND "lock.command"

enum states {
  state_home_needed,
  state_homing,
  state_homed_stalled,
  state_homed_retracting,
  state_unlocked,
  state_unlocking,
  state_locking,
  state_locked,
};

WiFiClient esp_client;
PubSubClient client(esp_client);
RotaryEncoder encoder(ENCODER_A_PIN, ENCODER_B_PIN, 3);

char message_buff[100];
bool debug = true;
long motor_position = 0;
long last_motor_position_on_homing_counter_overflow = 0;
long homing_loop_counter = 0;
int state = state_home_needed;
int last_published_state = -1;

void ICACHE_RAM_ATTR encoderISR() {
    encoder.readAB();
}

void setup() {
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    pinMode(MOTOR_EN_PIN, OUTPUT);
    
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
    digitalWrite(MOTOR_EN_PIN, HIGH);
    
    Serial.begin(115200);
    setup_wifi();
    client.setServer(MQTT_SERVER, 1883);
    client.setCallback(callback);
    encoder.begin();
    encoder.setPosition(0);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR, CHANGE);
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (client.connect("lock", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("OK");
            client.subscribe(MQTT_TOPIC_COMMAND);
        } else {
            Serial.print("FAILED: ");
            Serial.print(client.state());
            Serial.println(" Retrying...");
            delay(1000);
        }
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to '");
    Serial.print(WIFI_SSID);
    Serial.print("' .");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("OK");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }

    client.loop();

    long new_motor_position = encoder.getPosition();

    if (motor_position != new_motor_position) {
        motor_position = new_motor_position;
        Serial.print("pos: ");
        Serial.println(String(new_motor_position).c_str());

    }

    switch (state) {
        case state_locked:
        case state_unlocked:
            digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, HIGH);
            break;
        case state_unlocking:
            digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, LOW);
            if (motor_position >= 4 * MOTOR_STEPS_IN_ONE_TURN) {
                state = state_unlocked;
            }
            break;
        case state_locking:
            digitalWrite(MOTOR_A_PIN, LOW);
            digitalWrite(MOTOR_B_PIN, HIGH);
            if (motor_position < 0) {
                state = state_locked;
            }
            break;
        case state_home_needed:
            digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, HIGH);
            motor_position = 0;
            homing_loop_counter = 0;
            last_motor_position_on_homing_counter_overflow = motor_position;
            break;
        case state_homed_stalled:
            state = state_homed_retracting;
            encoder.setPosition(-MOTOR_HOME_RETRACT_COUNT);
            break;            
        case state_homing:
            //slower?
            digitalWrite(MOTOR_A_PIN, LOW);
            digitalWrite(MOTOR_B_PIN, HIGH);
            homing_loop_counter++;
            if (homing_loop_counter > 100000) {

                Serial.print("moved: ");
                Serial.println(String(last_motor_position_on_homing_counter_overflow - motor_position).c_str());
                
                if ((last_motor_position_on_homing_counter_overflow - motor_position) < MOTOR_HOME_STALL_THRESHOLD) {
                    state = state_homed_stalled;
                }
                homing_loop_counter = 0;
                last_motor_position_on_homing_counter_overflow = motor_position;
            }
            break;
        case state_homed_retracting:
            //slower?
            digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, LOW);
            if (motor_position >= 0) {
                state = state_locked;
            }
            break;
    }

    //digitalWrite(LED_PIN, LOW);

    if (state != last_published_state) {
        last_published_state = state;
        publish_state();
    }
}

void publish_state() {
    //client.publish(MQTT_TOPIC_CHANGED, String(newmotor_position).c_str(), true);
    Serial.print(" > ");
    switch (state) {
        case state_home_needed:
            Serial.println("state_home_needed");
            break;
        case state_homing:
            Serial.println("state_homing");
            break;
        case state_homed_stalled:
            Serial.println("state_homed_stalled");
            break;
        case state_homed_retracting:
            Serial.println("state_homed_retracting");
            break;
        case state_unlocked:
            Serial.println("state_unlocked");
            break;
        case state_unlocking:
            Serial.println("state_unlocking");
            break;
        case state_locking:
            Serial.println("state_locking");
            break;
        case state_locked:
            Serial.println("state_locked");
            break;
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    const char *cmd_lock = "lock";
    const char *cmd_unlock = "unlock";
    const char *cmd_home = "home";

    int i = 0;
    for (i = 0; i < length; i++) {
        message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    String msgString = String(message_buff);

    if (debug) {
        Serial.println("cmd> " + msgString);
    }

    if (msgString == cmd_lock) {
        state = state_locking;
    } else if (msgString == cmd_unlock) {
        state = state_unlocking;
    } else if (msgString == cmd_home) {
        state = state_homing;
    }
}
