#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "distanceSensors.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "Flood.h"
#include "labyrinth.h"
#include "firefighter.h"
#include "shared.h"

// ─────────────────────────────────────────
// TASK 1: Sensor Task (runs on Core 1)
// Reads the IMU and ToF sensor every 10ms
// and stores results in g_angle / g_frontDist
// ─────────────────────────────────────────
void sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // ALL sensors use I2C — IMU, front ToF, and IO expander (left/right walls)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_angle      = angle();           // BNO055 IMU
            g_frontDist  = checkFrontDist();  // VL53L1X front ToF
            g_leftWall   = leftWall();        // IO expander P7
            g_rightWall  = rightWall();       // IO expander P6
            xSemaphoreGive(i2cMutex);
        }

        // Wait until exactly 10ms has passed since last wake
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────
// TASK 2: Robot Task (runs on Core 0)
// This is your maze logic — same as before
// but now sensors are always being updated
// in the background by sensorTask
// ─────────────────────────────────────────
void robotTask(void *pvParameters) {
    // Wait 200ms for sensorTask to get its first readings
    vTaskDelay(pdMS_TO_TICKS(200));

    if (isFirefighter()) {
        while (1) vTaskDelay(pdMS_TO_TICKS(1000)); // do nothing
    }
    else if (isSpeedrun()) {
        Serial.println("Lightning McQueen mode");
        initialize();
        speedrun();
    }
    else if (isLabyrinth()) {
        Serial.println("Labyrinth mode");
        labyrinthLoop();
    }
    else {
        Serial.println("Exploration mode");
        initialize();
        runMaze('c');
    }

    // When the maze is done, delete this task (required — can't just return)
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────
// SETUP: runs once, creates tasks
// ─────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
    digitalWrite(LED_BUILTIN, HIGH);

    // Create the I2C mutex FIRST, before any I2C hardware setup
    i2cMutex = xSemaphoreCreateMutex();

    // Hardware setup (same as before)
    imuSetup();
    setupDistanceSensors();
    motorSetup();

    Serial.println("Starting tasks...");

    // Create sensorTask on Core 1, priority 3 (high)
    xTaskCreatePinnedToCore(
        sensorTask,   // the function
        "Sensors",    // name (for debugging)
        4096,         // stack size in bytes
        NULL,         // no parameters
        3,            // priority (higher number = higher priority)
        NULL,         // no handle needed
        1             // run on Core 1
    );

    // Create robotTask on Core 0, priority 2 (slightly lower)
    xTaskCreatePinnedToCore(
        robotTask,
        "Robot",
        8192,         // bigger stack — maze code is complex
        NULL,
        2,
        NULL,
        0             // run on Core 0
    );
}

// ─────────────────────────────────────────
// LOOP: we don't use this anymore
// Must still exist and must yield, not block
// ─────────────────────────────────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
