#include <Arduino.h>
#include "RotaryEncoder.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define LED_BLUE_PIN 2 //D4
#define LED_YELLOW_PIN 16 //D0
#define MOTOR_A_PIN 12 //D6
#define MOTOR_B_PIN 13 //D7
#define DOOR_SWITCH_PIN 14 //D5
#define BUTTON_PIN 0 //D3
#define ENCODER_A_PIN 4 //D2
#define ENCODER_B_PIN 5 //D1

#define MOTOR_HOME_RETRACT_COUNT 250
#define MOTOR_UNLATCH_STEPS 350
#define MOTOR_HOME_STALL_THRESHOLD 3
#define MOTOR_STEPS_IN_ONE_TURN 1364
#define MOTOR_PWM_FREQ 450
#define MOTOR_FULL_SPEED 1023
#define MOTOR_HOMING_SPEED 950

#define LEDS_FAST_BLINK_INTERVAL_MS 200
#define LEDS_SLOW_BLINK_INTERVAL_MS 600

#define WIFI_SSID "a"
#define WIFI_PASSWORD "b"
#define MQTT_SERVER "192.168.1.10"
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
  state_unlatched_waiting,
  state_unlatched_retracting
};

enum button_states {
  button_state_idle,
  button_state_maybe_short_pressed,
  button_state_inhibited_until_released
};

enum led_states {
  leds_off,
  leds_blue,
  leds_yellow,
  leds_blinking_blue,
  leds_blinking_yellow,
  leds_alternating
};

void mqtt_reconnect();
void update_motor_driver();
void motor_cw_on();
void motor_ccw_on();
void motor_stop();
void motor_cw_slow();
void motor_ccw_slow();
void update_motor_driver();
void publish_state();
void on_mqtt_message_received(char *topic, byte *payload, unsigned int length);
void on_button_short_press();
void on_button_long_press();
void wifi_setup();
void ICACHE_RAM_ATTR encoder_isr();

void update_leds_outputs();
const char *get_state_as_string();
void on_button_short_press();
void on_button_long_press();
void ota_setup_handler();

void update_lock_state();
void read_and_dispatch_button_events();
void toggle_blinking_leds_if_needed();
void update_leds_state();

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);
RotaryEncoder encoder(ENCODER_A_PIN, ENCODER_B_PIN, 3);

char message_buff[100];
bool debug = true;
long motor_position = 0;
long last_motor_position_on_homing_counter_overflow = 0;
long homing_loop_counter = 0;
long button_debounce_counter = 0;
long millis_at_last_led_toggle = 0;
long leds_blink_interval = LEDS_SLOW_BLINK_INTERVAL_MS;
int lock_state = state_home_needed;
int button_state = button_state_idle;
int leds_state = leds_off;
int last_published_state = -1;
unsigned long millis_at_unlatched;

void loop() {
    if (!mqtt_client.connected()) {
        mqtt_reconnect();
    }
    mqtt_client.loop();
    ArduinoOTA.handle();

    long new_motor_position = encoder.getPosition();
    if (motor_position != new_motor_position) {
        motor_position = new_motor_position;
        Serial.print(digitalRead(BUTTON_PIN) ? "B" : "b");
        Serial.print(digitalRead(DOOR_SWITCH_PIN) ? "D: " : "d: ");
        Serial.println(String(new_motor_position).c_str());

    }

    read_and_dispatch_button_events();
    update_lock_state();
    update_leds_state();

    if (lock_state != last_published_state) {
        last_published_state = lock_state;
        publish_state();
        update_motor_driver();
        update_leds_outputs();
    }

    toggle_blinking_leds_if_needed();
}

void update_leds_state() {
    switch (lock_state) {
        case state_locking:
            leds_state = leds_blinking_yellow;
            leds_blink_interval = LEDS_FAST_BLINK_INTERVAL_MS;
            break;
        case state_locked:
            leds_state = leds_yellow;
            break;
        case state_unlocking:
            leds_state = leds_blinking_blue;
            leds_blink_interval = LEDS_FAST_BLINK_INTERVAL_MS;
            break;
        case state_unlocked:
            leds_state = leds_blue;
            break;
        case state_home_needed:
        case state_homing:
            leds_state = leds_alternating;
            leds_blink_interval = LEDS_SLOW_BLINK_INTERVAL_MS;
            break;
        default:
            leds_state = leds_off;
    }
}

void toggle_blinking_leds_if_needed() {
    if (leds_state < leds_blinking_blue) {
        return;
    }

    unsigned long now = millis();

    if (now - millis_at_last_led_toggle > leds_blink_interval) {
        millis_at_last_led_toggle = now;
        update_leds_outputs();
    }
}

void read_and_dispatch_button_events() {
    if (!digitalRead(BUTTON_PIN)) {
        button_debounce_counter++;

        if (button_state == button_state_idle && button_debounce_counter > 30) {
            button_state = button_state_maybe_short_pressed;
        }

        if (button_state == button_state_maybe_short_pressed && button_debounce_counter > 20000) {
            on_button_long_press();
            button_state = button_state_inhibited_until_released;
            button_debounce_counter = 30;
        }

    } else {
        if (button_debounce_counter > 0) {
            button_debounce_counter--;
        }

        if (button_state == button_state_maybe_short_pressed && button_debounce_counter < 10) {
            on_button_short_press();
            button_state = button_state_idle;
            button_debounce_counter = 0;
        }

        if (button_state == button_state_inhibited_until_released && button_debounce_counter == 0) {
            button_state = button_state_idle;
        }
    }
}

void update_lock_state() {
    switch (lock_state) {
        case state_unlatching:
            if (motor_position >= MOTOR_UNLATCH_STEPS + 2 * MOTOR_STEPS_IN_ONE_TURN) {
                lock_state = state_unlatched_waiting;
                millis_at_unlatched = millis();
            }
            break;
        case state_unlatched_waiting:
            if (millis() - millis_at_unlatched > 500) {
                lock_state = state_unlatched_retracting;
            }
            break;
        case state_unlatched_retracting:
            if (motor_position <= 2 * MOTOR_STEPS_IN_ONE_TURN) {
                lock_state = state_unlocked;
            }
            break;
        case state_unlocking:
            leds_state = leds_blinking_blue;
            if (motor_position >= 2 * MOTOR_STEPS_IN_ONE_TURN) {
                lock_state = state_unlocked;
            }
            break;
        case state_locking:
            leds_state = leds_alternating;
            if (motor_position < 0) {
                lock_state = state_locked;
            }
            break;
        case state_home_needed:
            leds_state = leds_alternating;
            motor_position = 0;
            homing_loop_counter = 0;
            last_motor_position_on_homing_counter_overflow = 0;
            break;
        case state_homed_stalled:
            lock_state = state_homed_retracting;
            encoder.setPosition(-MOTOR_HOME_RETRACT_COUNT);
            break;
        case state_homing:
            homing_loop_counter++;
            if (homing_loop_counter > 60000) {
                Serial.print("moved: ");
                Serial.println(String(last_motor_position_on_homing_counter_overflow - motor_position).c_str());
                if ((last_motor_position_on_homing_counter_overflow - motor_position) < MOTOR_HOME_STALL_THRESHOLD) {
                    lock_state = state_homed_stalled;
                }
                homing_loop_counter = 50000;
                last_motor_position_on_homing_counter_overflow = motor_position;
            }
            break;
        case state_homed_retracting:
            if (motor_position >= 0) {
                lock_state = state_locked;
            }
            break;
    }
}

void on_button_long_press() {
    switch (lock_state) {
        case state_unlocked:
            lock_state = state_locking;
            break;
        case state_locked:
            lock_state = state_unlocking;
            break;
        case state_home_needed:
            lock_state = state_homing;
    }
}

void on_button_short_press() {
    if (lock_state == state_locked) {
        lock_state = state_unlocking;
    }
}

void update_leds_outputs() {
    switch (leds_state) {
        case leds_off:
            digitalWrite(LED_YELLOW_PIN, LOW);
            digitalWrite(LED_BLUE_PIN, LOW);
            break;
        case leds_yellow:
            digitalWrite(LED_YELLOW_PIN, HIGH);
            digitalWrite(LED_BLUE_PIN, LOW);
            break;
        case leds_blue:
            digitalWrite(LED_YELLOW_PIN, LOW);
            digitalWrite(LED_BLUE_PIN, HIGH);
            break;
        case leds_blinking_blue:
            digitalWrite(LED_YELLOW_PIN, LOW);
            digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
            break;
        case leds_blinking_yellow:
            digitalWrite(LED_BLUE_PIN, LOW);
            digitalWrite(LED_YELLOW_PIN, !digitalRead(LED_YELLOW_PIN));
            break;
        case leds_alternating:
            digitalWrite(LED_BLUE_PIN, digitalRead(LED_YELLOW_PIN));
            digitalWrite(LED_YELLOW_PIN, !digitalRead(LED_BLUE_PIN));
    }
}

void motor_cw_on() {
    digitalWrite(MOTOR_A_PIN, LOW);
    analogWrite(MOTOR_B_PIN, MOTOR_FULL_SPEED);
}

void motor_ccw_on() {
    analogWrite(MOTOR_A_PIN, MOTOR_FULL_SPEED);
    digitalWrite(MOTOR_B_PIN, LOW);
}

void motor_cw_slow() {
    digitalWrite(MOTOR_A_PIN, LOW);
    analogWrite(MOTOR_B_PIN, MOTOR_HOMING_SPEED);
}

void motor_ccw_slow() {
    analogWrite(MOTOR_A_PIN, MOTOR_HOMING_SPEED);
    digitalWrite(MOTOR_B_PIN, LOW);
}

void motor_stop() {
    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, HIGH);
}

void update_motor_driver() {
    switch (lock_state) {
        case state_home_needed:
        case state_homed_stalled:
        case state_locked:
        case state_unlocked:
        case state_unlatched_waiting:
            motor_stop();
            break;
        case state_unlocking:
            motor_ccw_on();
            break;
        case state_locking:
            motor_cw_on();
            break;
        case state_homing:
            motor_cw_slow();
            break;
        case state_homed_retracting:
            motor_ccw_on();
            break;
        case state_unlatching:
            motor_ccw_on();
            break;
        case state_unlatched_retracting:
            motor_cw_on();
            break;
    }
}

void publish_state() {
    const char *state_string = get_state_as_string();

    Serial.print("> ");
    Serial.println(state_string);
    mqtt_client.publish(MQTT_TOPIC_STATUS, String(state_string).c_str(), true);
}

const char *get_state_as_string() {
    switch (lock_state) {
        case state_home_needed:
            return "state_home_needed";
        case state_homing:
            return "state_homing";
        case state_homed_stalled:
            return "state_homed_stalled";
        case state_homed_retracting:
            return "state_homed_retracting";
        case state_unlocked:
            return "state_unlocked";
        case state_unlocking:
            return "state_unlocking";
        case state_locking:
            return "state_locking";
        case state_locked:
            return "state_locked";
        case state_unlatching:
            return "state_unlatching";
        case state_unlatched_waiting:
            return "state_unlatched_waiting";
        case state_unlatched_retracting:
            return "state_unlatched_retracting";
    }
}

void on_mqtt_message_received(char *topic, byte *payload, unsigned int length) {
    const char *cmd_lock = "lock";
    const char *cmd_unlock = "unlock";
    const char *cmd_home = "home";
    const char *cmd_unlatch = "unlatch";
    const char *cmd_adj_ccw = "accw";
    const char *cmd_adj_cw = "acw";

    int i = 0;
    for (i = 0; i < length; i++) {
        message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    String received_message = String(message_buff);

    if (debug) {
        Serial.println("cmd> " + received_message);
    }

    if (received_message == cmd_adj_ccw) {
        motor_ccw_on();
        delay(200);
        motor_stop();
    } else if (received_message == cmd_adj_cw) {
        motor_cw_on();
        delay(200);
        motor_stop();
    } else if (received_message == cmd_lock) {
        if (lock_state == state_unlocked) {
            lock_state = state_locking;
        }
    } else if (received_message == cmd_unlock) {
        if (lock_state == state_locked) {
            lock_state = state_unlocking;
        }
    } else if (received_message == cmd_home) {
        lock_state = state_homing;
        motor_position = 0;
        homing_loop_counter = 0;
    } else if (received_message == cmd_unlatch) {
        if (lock_state == state_unlocked) {
            lock_state = state_unlatching;
        }
    }
}

void ICACHE_RAM_ATTR encoder_isr() {
    encoder.readAB();
}

void setup() {
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);
    pinMode(DOOR_SWITCH_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_YELLOW_PIN, OUTPUT);
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);

    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
    analogWriteFreq(MOTOR_PWM_FREQ);

    Serial.begin(115200);
    wifi_setup();
    mqtt_client.setServer(MQTT_SERVER, 1883);
    mqtt_client.setCallback(on_mqtt_message_received);
    mqtt_reconnect();

    encoder.begin();
    encoder.setPosition(0);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_isr, CHANGE);
}

void wifi_setup() {
    digitalWrite(LED_YELLOW_PIN, HIGH);
    delay(10);
    Serial.println();
    Serial.print("Connecting to '");
    Serial.print(WIFI_SSID);
    Serial.print("' .");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        digitalWrite(LED_YELLOW_PIN, !digitalRead(LED_YELLOW_PIN));
        Serial.print(".");
    }
    digitalWrite(LED_YELLOW_PIN, HIGH);

    Serial.println("OK");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    ota_setup_handler();
    digitalWrite(LED_YELLOW_PIN, LOW);
}

void ota_setup_handler() {
    Serial.println("Starting OTA handler...");

    ArduinoOTA.onStart([]() {
      digitalWrite(MOTOR_A_PIN, LOW);
      digitalWrite(MOTOR_B_PIN, LOW);
      digitalWrite(LED_BLUE_PIN, LOW);
      digitalWrite(LED_YELLOW_PIN, LOW);
      timer1_disable();
      timer1_detachInterrupt();

      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
          type = "sketch";
      } else { // U_FS
          type = "filesystem";
      }

      Serial.println("OTA Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA End");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
      digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
          Serial.println("OTA Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
          Serial.println("OTA Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
          Serial.println("OTA Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("OTA Receive Failed");
      } else if (error == OTA_END_ERROR) {
          Serial.println("OTA End Failed");
      }
    });

    ArduinoOTA.begin();
    ArduinoOTA.setRebootOnSuccess(true);
    Serial.println("OTA Initialized");
}

void mqtt_reconnect() {
    digitalWrite(LED_BLUE_PIN, LOW);
    while (!mqtt_client.connected()) {
        //? wifi_setup();
        Serial.print("Connecting to MQTT broker...");
        if (mqtt_client.connect("lock", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("Connected, subscribing...");
            mqtt_client.subscribe(MQTT_TOPIC_COMMAND);
            digitalWrite(LED_BLUE_PIN, HIGH);
        } else {
            Serial.print("FAILED: ");
            Serial.print(mqtt_client.state());
            Serial.println(" Retrying...");
            digitalWrite(LED_BLUE_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_BLUE_PIN, LOW);
        }
    }
    Serial.println("OK: MQTT connected & subscribed");
    digitalWrite(LED_BLUE_PIN, HIGH);
}