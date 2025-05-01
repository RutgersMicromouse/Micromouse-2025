#pragma once
// https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#software-preparation
// https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/#turn-on-the-psram-option
// https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera/tree/main
#include<Arduino.h>
#include "esp_camera.h"
#include "motors.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

void photo_save();

void cameraSetup();

#define ASCII_CHARS "#@%*+=-:. "

void imageToAscii();
void followEdge();


