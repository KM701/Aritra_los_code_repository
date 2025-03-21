#include <FlySkyIBus.h>
#include <Servo.h>

Servo thrustersb;
Servo thrusterpt;

int CH1, CH2, CH7, CH8, CH4;
int thrusterptvalue = 1500, thrustersbvalue = 1500;
bool autonomyMode = false;
int autoThrusterPT = 1500, autoThrusterSB = 1500;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    IBus.begin(Serial1);

    thrustersb.attach(12);
    thrusterpt.attach(13);

    Serial.println("🚀 System Initialized");
}

void loop() {
    IBus.loop();  

    // Read RC Channels
    CH1 = IBus.readChannel(0);
    CH2 = IBus.readChannel(1);
    CH4 = IBus.readChannel(3);
    CH7 = IBus.readChannel(6);
    CH8 = IBus.readChannel(7);

    // **Mode Selection**
    if (CH8 > 1500) {  
        autonomyMode = false;  // RC Mode enabled when CH8 is HIGH
    } else if (CH8 < 1200) {  
        autonomyMode = true;   // Autonomy mode when CH8 is LOW
    }

    // **Autonomy Mode**
    if (autonomyMode) {
        Serial.println("🟢 Autonomy Mode Active: Ignoring RF");
        
        // **Read Serial Commands**
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            Serial.print("Received: "); Serial.println(input);

            int left, right;
            if (sscanf(input.c_str(), "L%d R%d", &left, &right) == 2) {
                autoThrusterPT = constrain(left, 1400, 1650);
                autoThrusterSB = constrain(right, 1400, 1650);
            }
        }
        thrusterptvalue = autoThrusterPT;
        thrustersbvalue = autoThrusterSB;

    } else {
        Serial.println("🔵 RC Mode Active: Using RF Input");
        
        // **Manual RC Mode**
        if (abs(CH4 - 1500) > 10) {  // Deadband for turning
            thrusterptvalue = CH4;
            thrustersbvalue = 3000 - thrusterptvalue;
        } else {
            thrusterptvalue = CH2 + CH1 - 1500;
            thrustersbvalue = CH2 - CH1 + 1500;
        }

        // **Dead Zone Fix**
        if (abs(thrusterptvalue - 1500) < 15 && abs(thrustersbvalue - 1500) < 15) {
            thrusterptvalue = 1500;
            thrustersbvalue = 1500;
        } else {
            thrusterptvalue = constrain(thrusterptvalue, 1400, 1650);
            thrustersbvalue = constrain(thrustersbvalue, 1400, 1650);
        }
    }

    // **Apply to Thrusters**
    thrustersb.writeMicroseconds(thrustersbvalue);
    thrusterpt.writeMicroseconds(thrusterptvalue);

    Serial.print("Thruster PT: "); Serial.print(thrusterptvalue);
    Serial.print(" | Thruster SB: "); Serial.println(thrustersbvalue);

    delay(50);
}

