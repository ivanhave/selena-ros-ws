/*
This code is to be uploaded on the arduino nano that sits on 
the servo side. The setup includes the Arduino Nano, MCP5215 can board
and MG996R servo motor.
*/

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

// ── Configuration — edit here only ──────────────────────
const int     SPI_CS_PIN   = 10;
const int     SERVO_PIN    = 6;
const uint8_t GRIPPER_ID   = 0x090;  // CAN node ID
const int     ANGLE_MIN    = 0;
const int     ANGLE_MAX    = 55;
const int     DETACH_DELAY = 500;    // ms to wait after move before detaching
// ────────────────────────────────────────────────────────

MCP_CAN CAN(SPI_CS_PIN);
Servo   gripperServo;

void setup()
{
    Serial.begin(115200);

    // Move to home position on start
    gripperServo.attach(SERVO_PIN);
    gripperServo.write(ANGLE_MIN);
    delay(DETACH_DELAY);
    gripperServo.detach();
    Serial.println("Gripper homed and detached.");

    // Init CAN bus
    while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
        Serial.println("CAN Init Failed, retrying...");
        delay(1000);
    }
    CAN.setMode(MCP_NORMAL);

    Serial.print("Gripper ready. Listening on CAN ID: 0x");
    Serial.println(GRIPPER_ID, HEX);
}

void loop()
{
    unsigned long canId = 0;
    unsigned char len   = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL != CAN.checkReceive()) return;

    CAN.readMsgBuf(&canId, &len, buf);

    // Ignore frames not meant for us
    if (canId != GRIPPER_ID) return;

    int angle = buf[0];

    // Validate angle range
    if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
        Serial.print("Angle out of range (");
        Serial.print(angle);
        Serial.print("). Valid range: ");
        Serial.print(ANGLE_MIN);
        Serial.print(" - ");
        Serial.println(ANGLE_MAX);
        return;
    }

    // Move servo
    Serial.print("Moving to: ");
    Serial.println(angle);

    gripperServo.attach(SERVO_PIN);
    gripperServo.write(angle);
    delay(DETACH_DELAY);
    gripperServo.detach();

    Serial.println("Move complete. Servo detached.");

    // Send confirmation back
    unsigned char reply[1] = { (unsigned char)angle };
    CAN.sendMsgBuf(GRIPPER_ID, 0, 1, reply);
}