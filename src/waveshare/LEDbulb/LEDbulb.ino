#include <Arduino.h>
int green_pin_pwm = 18;
int red_pin_pwm = 27;
int blue_pin_pwm = 17;
int green_pin_dir = 4;
int red_pin_dir = 12;
int blue_pin_dir = 16;

void setup() {
    pinMode(red_pin_dir, OUTPUT);
    pinMode(blue_pin_dir, OUTPUT);
    pinMode(green_pin_dir, OUTPUT);
    pinMode(green_pin_pwm, OUTPUT);
    pinMode(red_pin_pwm, OUTPUT);
    pinMode(blue_pin_pwm, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    while (Serial.available() > 0) {
        String receivedString = Serial.readStringUntil('\n');
        int state = receivedString.toInt();
        if (state >= 0) {
            Serial.print(state);
            if (state == 1){
                Serial.print("RED");

                digitalWrite(blue_pin_pwm, LOW);

                digitalWrite(green_pin_pwm, LOW);

                digitalWrite(red_pin_dir, LOW);
                digitalWrite(red_pin_pwm, HIGH);

            }else if (state == 2) {
              Serial.print("GREEN");

                digitalWrite(blue_pin_pwm, LOW);

                digitalWrite(red_pin_pwm, LOW);

                digitalWrite(green_pin_dir, LOW);
                digitalWrite(green_pin_pwm, HIGH);
                delay(200);
                digitalWrite(green_pin_pwm, LOW);
                delay(200);
            }
            else if (state == 0) {
              Serial.print("BLUE");

                digitalWrite(green_pin_pwm, LOW);

                digitalWrite(red_pin_pwm, LOW);
                
                digitalWrite(blue_pin_dir, LOW);
                digitalWrite(blue_pin_pwm, HIGH);
            }
        }
    }
}
