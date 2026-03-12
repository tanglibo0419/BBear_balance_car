#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include <MPU6050_light.h>
#include <math.h>
#include "BluetoothSerial.h"

//==================== 对象 ====================

BluetoothSerial SerialBT;
MPU6050 mpu(Wire);

ESP32Encoder encoderL(0);
ESP32Encoder encoderR(1);

//==================== 引脚 ====================

const int AIN1 = 12, AIN2 = 13, PWMA = 14;
const int BIN1 = 26, BIN2 = 25, PWMB = 27;
const int STBY = 5;

const int ENC_LA = 15, ENC_LB = 2;
const int ENC_RA = 16, ENC_RB = 4;

//==================== PID 参数 ====================

volatile float Kp = 350;
volatile float Kd = 1.0;

volatile float Kp_speed = 0.1;
volatile float Ki_speed = 0.005;

volatile float alpha = 0.98;
volatile float angleOffset = 1.3;

volatile int motor_deadzone = 20;

//==================== 遥控变量 ====================

volatile float targetSpeedCmd = 0;   // 遥控目标速度
volatile float targetSpeed = 0;      // 平滑后的速度
volatile float targetTurn = 0;
volatile float speedLevel = 20;

//==================== 状态变量 ====================

float angle = 0;
float gyroX = 0;

float targetAngle = 0;

float speedIntegral = 0;
float speed = 0;

int pwmL = 0;
int pwmR = 0;

unsigned long lastUpdateTime = 0;

//==================== PWM ====================

const int freq = 5000;
const int chA = 0;
const int chB = 1;
const int resolution = 11;

//==================== 定时器 ====================

hw_timer_t *timer = NULL;
SemaphoreHandle_t timerSemaphore;

//==================== 电机控制 ====================

void setMotorL(int pwm)
{
    if (pwm == 0)
    {
        ledcWrite(chA, 0);
        return;
    }

    int final_pwm = (pwm > 0) ? (pwm + motor_deadzone) : (pwm - motor_deadzone);
    final_pwm = constrain(final_pwm, -2048, 2048);

    digitalWrite(AIN1, final_pwm > 0);
    digitalWrite(AIN2, final_pwm <= 0);

    ledcWrite(chA, abs(final_pwm));
}

void setMotorR(int pwm)
{
    if (pwm == 0)
    {
        ledcWrite(chB, 0);
        return;
    }

    int final_pwm = (pwm > 0) ? (pwm + motor_deadzone) : (pwm - motor_deadzone);
    final_pwm = constrain(final_pwm, -2048, 2048);

    digitalWrite(BIN1, final_pwm > 0);
    digitalWrite(BIN2, final_pwm <= 0);

    ledcWrite(chB, abs(final_pwm));
}

//==================== 蓝牙遥控解析 ====================

void handleBluetoothControl(char c)
{
    switch (c)
    {
        case 'F':
            targetSpeedCmd = speedLevel;
            targetTurn = 0;
            break;

        case 'B':
            targetSpeedCmd = -speedLevel;
            targetTurn = 0;
            break;

        case 'L':
            targetSpeedCmd = speedLevel;
            targetTurn = 50;
            break;

        case 'R':
            targetSpeedCmd = speedLevel;
            targetTurn = -50;
            break;

        case 'S':
            targetSpeedCmd = 0;
            targetTurn = 0;
            break;

        case '1': speedLevel = 10; break;
        case '2': speedLevel = 20; break;
        case '3': speedLevel = 30; break;
        case '4': speedLevel = 40; break;
        case '5': speedLevel = 50; break;
        case '6': speedLevel = 60; break;
        case '7': speedLevel = 70; break;
    }
}

//==================== 执行调参指令 ====================

void executeCommand(char cmd, float val)
{
    switch (toupper(cmd))
    {
        case 'P': Kp = val; break;
        case 'D': Kd = val; break;
        case 'S': Kp_speed = val; break;
        case 'I': Ki_speed = val; break;
        case 'A': alpha = constrain(val, 0, 1); break;
        case 'O': angleOffset = val; break;
        case 'M': motor_deadzone = (int)val; break;
        case 'V': targetSpeed = val; break;
    }
}

//==================== 串口调参 ====================

void handleSerialTuning()
{
    static String btBuffer = "";

    while (SerialBT.available())
    {
        char c = SerialBT.read();

        handleBluetoothControl(c);

        if (c != '\0')
            Serial.write(c);

        if (c == '\n' || c == '\r')
        {
            if (btBuffer.length() > 0)
            {
                char cmd = btBuffer[0];
                float val = btBuffer.substring(1).toFloat();
                executeCommand(cmd, val);
                btBuffer = "";
            }
        }
        else if (c != '\0')
        {
            btBuffer += c;
        }
    }

    if (Serial.available())
    {
        char cmd = Serial.read();
        float val = Serial.parseFloat();

        executeCommand(cmd, val);
        Serial.printf("\n[USB] %c = %.4f\n", toupper(cmd), val);
    }
}

//==================== 定时器 ====================

void IRAM_ATTR onTimer()
{
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

//==================== 控制环 ====================

void TaskControl(void *pvParameters)
{
    float gyroFilter = 0;

    while (true)
    {
        if (xSemaphoreTake(timerSemaphore, portMAX_DELAY))
        {
            Wire.beginTransmission(0x68);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 14);

            int16_t ax = (Wire.read() << 8) | Wire.read();
            int16_t ay = (Wire.read() << 8) | Wire.read();
            int16_t az = (Wire.read() << 8) | Wire.read();

            Wire.read(); Wire.read();

            int16_t gx = (Wire.read() << 8) | Wire.read();

            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime) / 1000.0;

            if (dt <= 0 || dt > 0.1)
                dt = 0.005;

            lastUpdateTime = currentTime;

            gyroX = gx / 131.0;

            float accel_angle = atan2((float)ay, (float)az) * 180.0 / M_PI + angleOffset;

            angle = alpha * (angle + gyroX * dt) + (1 - alpha) * accel_angle;

            gyroFilter = 0.9 * gyroFilter + 0.1 * gyroX;

            float vertical = Kp * (targetAngle - angle) + Kd * gyroFilter;

            int turn = (int)targetTurn;

            pwmL = vertical + turn;
            pwmR = vertical - turn;

            if (abs(angle) > 20)
            {
                setMotorL(0);
                setMotorR(0);
                speedIntegral = 0;
            }
            else
            {
                setMotorL(pwmL);
                setMotorR(pwmR);
            }
        }
    }
}

//==================== 速度环 ====================

void TaskSpeed(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();

    long lastL = 0;
    long lastR = 0;

    while (true)
    {
        // ===== 平滑加速 =====
        float accel = 2.5; // 每周期最大变化量

        if (targetSpeed < targetSpeedCmd)
        {
            targetSpeed += accel;
            if (targetSpeed > targetSpeedCmd)
                targetSpeed = targetSpeedCmd;
        }
        else if (targetSpeed > targetSpeedCmd)
        {
            targetSpeed -= accel * 2;
            if (targetSpeed < targetSpeedCmd)
                targetSpeed = targetSpeedCmd;
        }

        long nowL = encoderL.getCount();
        long nowR = encoderR.getCount();

        speed = ((nowL - lastL) + (nowR - lastR)) / 2.0;

        lastL = nowL;
        lastR = nowR;

        float error = targetSpeed - speed;

        speedIntegral = constrain(speedIntegral + error * 0.02, -250, 250);

        static float angleFilter = 0;

        float out = Kp_speed * error + Ki_speed * speedIntegral;

        // 将 0.8/0.2 改为 0.9/0.1 或者 0.95/0.05
        angleFilter = 0.9 * angleFilter + 0.1 * out;

        targetAngle = angleFilter;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));
    }
}

//==================== 初始化 ====================

void setup()
{
    Serial.begin(115200);

    SerialBT.begin("ESP32_Balance_Car");

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);

    ledcSetup(chA, freq, resolution);
    ledcAttachPin(PWMA, chA);

    ledcSetup(chB, freq, resolution);
    ledcAttachPin(PWMB, chB);

    encoderL.attachFullQuad(ENC_LA, ENC_LB);
    encoderR.attachFullQuad(ENC_RA, ENC_RB);

    Wire.begin();
    Wire.setClock(400000);

    mpu.begin();
    mpu.calcOffsets();

    timerSemaphore = xSemaphoreCreateBinary();

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);

    xTaskCreatePinnedToCore(TaskControl, "control", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskSpeed, "speed", 4096, NULL, 4, NULL, 1);
}

//==================== 主循环 ====================

void loop()
{
    handleSerialTuning();

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 200)
    {
        if (SerialBT.hasClient())
        {
            char buf[180];

            sprintf(buf,
                    "Ang:%.2f Tgt:%.2f Spd:%.1f CmdSpd:%.0f Turn:%.0f P:%.0f D:%.2f\n",
                    angle,
                    targetAngle,
                    speed,
                    targetSpeed,
                    targetTurn,
                    Kp,
                    Kd);

            SerialBT.print(buf);
        }

        lastPrint = millis();
    }
}