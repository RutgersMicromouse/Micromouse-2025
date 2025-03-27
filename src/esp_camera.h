// https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#software-preparation
// https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/#turn-on-the-psram-option
// https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera/tree/main

#include "esp_camera.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

void photo_save() {
  /*
  typedef struct {
    uint8_t * buf;              // Pointer to the pixel data
    size_t len;                 // Length of the buffer in bytes
    size_t width;               // Width of the buffer in pixels
    size_t height;              // Height of the buffer in pixels
    pixformat_t format;         // Format of the pixel data
    struct timeval timestamp;   // Timestamp since boot of the first DMA buffer of the frame
  } camera_fb_t;
  */

  camera_fb_t * fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }

  Serial.println(fb->height);
  Serial.println(fb->width);
  Serial.println(fb->len);
  Serial.println(fb->format);

  // Release image buffer
  esp_camera_fb_return(fb);
}



void setup() {
  Serial.begin(115200);

  static camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;


  /*
  QVGA - Quarter Video Graphics Array, with a resolution of 320 x 240 pixels.
  CIF - Common Intermediate Format, with a resolution of 352 x 288 pixels and a 4:3 aspect ratio.
  VGA - Video Graphics Array, with a resolution of 640 x 480 pixels and a 4:3 aspect ratio. 
  SVGA - Super Video Graphics Array, with a resolution of 800 x 600 pixels and a 4:3 aspect ratio. 
  XGA - eXtended Graphics Array, with a resolution of 1024 x 768 pixels and a 4:3 aspect ratio. 
  SXGA - Super eXtended Graphics Array, with a resolution of 1280 x 1024 pixels and a 4:3 aspect ratio. 
  UXGA - Ultra eXtended Graphics Array, with a resolution of 1600 x 1200 pixels and a 4:3 aspect ratio. 
  */
  config.frame_size = FRAMESIZE_QVGA; // image resolution, any resolution above SVGA causes a boot loop, since it overflows the frame buffer


  /*
  // esp_camera.h -> sensor.h  
  typedef enum {
      PIXFORMAT_RGB565,    // 2BPP/RGB565, 0
      PIXFORMAT_YUV422,    // 2BPP/YUV422, 1
      PIXFORMAT_YUV420,    // 1.5BPP/YUV420, 2
      PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE, 3
      PIXFORMAT_JPEG,      // JPEG/COMPRESSED, 4
      PIXFORMAT_RGB888,    // 3BPP/RGB888, 5
      PIXFORMAT_RAW,       // RAW, 6
      PIXFORMAT_RGB444,    // 3BP2P/RGB444, 7
      PIXFORMAT_RGB555,    // 3BP2P/RGB555, 8
  } pixformat_t;
  */
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Grayscale format

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM for frame buffers
  config.jpeg_quality = 0;
  config.fb_count = 1;

  // Initialize the camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }
  
  Serial.println("Camera init success");
}

 
#define ASCII_CHARS "#@%*+=-:. "



void loop() {
  // Capture a frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Process the frame
  Serial.printf("Captured frame %dx%d, size: %d bytes\n", fb->width, fb->height, fb->len);

  // Access the image data
  uint8_t* image_data = fb->buf;

  // Iterate over all pixels to convert to ASCII
  for (int y = 0; y < fb->height; y++) { // Iterate over rows
    for (int x = 0; x < fb->width; x++) { // Iterate over columns
      uint8_t gray = image_data[y * fb->width + x];

      
      char ascii_char = ASCII_CHARS[gray / 25]; // 256 / 10 = 25, so 0-25 -> 0, 26-50 -> 1, ..., 226-255 -> 9
      Serial.print(ascii_char);
      
  }
  Serial.println(); // Newline at the end of each row
}

  Serial.println(); // Newline at the end of each row
  Serial.println(); // Newline at the end of each row
  Serial.println(); // Newline at the end of each row

  // Return the frame buffer back to the driver
  esp_camera_fb_return(fb);

  // Delay before capturing next frame
  delay(500); // 1 second delay

}