#include <Arduino.h>
#include "RotaryEncoder.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define LED_PIN   2 //D4

#define MOTOR_A_PIN 12 //D6
#define MOTOR_B_PIN 13 //D7
#define MOTOR_EN_PIN 14 //D5

#define ENCODER_A_PIN 5 //D1
#define ENCODER_B_PIN 4 //D2

#define MOTOR_HOME_RETRACT_COUNT 180
#define MOTOR_HOME_STALL_THRESHOLD 5
#define MOTOR_STEPS_IN_ONE_TURN 1364
#define MOTOR_PWM_FREQ 200
#define MOTOR_FULL_SPEED 1023
#define MOTOR_HOMING_SPEED 500

#define WIFI_SSID "sdfgsdfg"
#define WIFI_PASSWORD "sdgfsdfg"
#define MQTT_SERVER "sdfgsdf"
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
  state_unlatching,
  state_unlatched_stalled,
  state_unlatched_retracting
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

void ICACHE_RAM_ATTR
encoderISR() {
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
    digitalWrite(MOTOR_EN_PIN, LOW);
    analogWriteFreq(MOTOR_PWM_FREQ);

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

void motor_cw_on() {
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, HIGH);
}

void motor_ccw_on() {
    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, LOW);
}

void motor_stop() {
    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, HIGH);
}

void update_motor_driver() {
    switch (state) {
        case state_home_needed:
        case state_homed_stalled:
        case state_locked:
        case state_unlocked:
        case state_unlatched_stalled:
            motor_stop();
            break;
        case state_unlocking:
            analogWrite(MOTOR_EN_PIN, MOTOR_FULL_SPEED);
            motor_ccw_on();
            break;
        case state_locking:
            analogWrite(MOTOR_EN_PIN, MOTOR_FULL_SPEED);
            motor_cw_on();
            break;
        case state_homing:
            analogWrite(MOTOR_EN_PIN, MOTOR_HOMING_SPEED);
            motor_cw_on();
            break;
        case state_homed_retracting:
            analogWrite(MOTOR_EN_PIN, MOTOR_HOMING_SPEED);
            motor_ccw_on();
            break;
        case state_unlatching:
            analogWrite(MOTOR_EN_PIN, MOTOR_FULL_SPEED);
            motor_ccw_on();
            break;
        case state_unlatched_retracting:
            analogWrite(MOTOR_EN_PIN, MOTOR_HOMING_SPEED);
            motor_cw_on();
            break;
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
        case state_unlatching:
            Serial.println("state_unlatching");
            break;
        case state_unlatched_stalled:
            Serial.println("state_unlatched_stalled");
            break;
        case state_unlatched_retracting:
            Serial.println("state_unlatched_retracting");
            break;
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    const char *cmd_lock = "lock";
    const char *cmd_unlock = "unlock";
    const char *cmd_home = "home";
    const char *cmd_unlatch = "unlatch";

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
        if (state == state_unlocked) {
            state = state_locking;
        }
    } else if (msgString == cmd_unlock) {
        if (state == state_locked) {
            state = state_unlocking;
        }
    } else if (msgString == cmd_home) {
        state = state_homing;
        motor_position = 0;
        homing_loop_counter = 0;
    } else if (msgString == cmd_unlatch) {
        if (state == state_unlocked) {
            state = state_unlatching;
        }
    }
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
        case state_unlocking:
            if (motor_position >= 2 * MOTOR_STEPS_IN_ONE_TURN) {
                state = state_unlocked;
            }
            break;
        case state_locking:
            if (motor_position < 0) {
                state = state_locked;
            }
            break;
        case state_home_needed:
            motor_position = 0;
            homing_loop_counter = 0;
            last_motor_position_on_homing_counter_overflow = 0;
            break;
        case state_homed_stalled:
            state = state_homed_retracting;
            encoder.setPosition(-MOTOR_HOME_RETRACT_COUNT);
            break;
        case state_homing:
            homing_loop_counter++;
            if (homing_loop_counter > 60000) {
                Serial.print("moved: ");
                Serial.println(String(last_motor_position_on_homing_counter_overflow - motor_position).c_str());
                if ((last_motor_position_on_homing_counter_overflow - motor_position) < MOTOR_HOME_STALL_THRESHOLD) {
                    state = state_homed_stalled;
                }
                homing_loop_counter = 50000;
                last_motor_position_on_homing_counter_overflow = motor_position;
            }
            break;
        case state_homed_retracting:
            if (motor_position >= 0) {
                state = state_locked;
            }
            break;
    }

    if (state != last_published_state) {
        last_published_state = state;
        publish_state();
        update_motor_driver();
    }
}
