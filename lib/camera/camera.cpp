#include "camera.h"

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

void cameraSetup() {
    
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

void imageToAscii() {

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
  for (int y = 0; y < fb->height - 90; y++) { // Iterate over rows
    for (int x = 0; x < fb->width; x++) { // Iterate over columns
      uint8_t gray = image_data[y * fb->width + x];
      if (gray > 180) {
        gray = 255;
      }
      if (gray < 180){
        gray = 0;
      }
      char ascii_char = ASCII_CHARS[gray / 25]; // 256 / 10 = 25, so 0-25 -> 0, 26-50 -> 1, ..., 226-255 -> 9
      Serial.print(ascii_char);
      
      
  }
  Serial.println("|"); // Newline at the end of each row
}

  Serial.println(); // Newline at the end of each row
  Serial.println(); // Newline at the end of each row
  Serial.println(); // Newline at the end of each row

  // Return the frame buffer back to the driver
  esp_camera_fb_return(fb);

  // Delay before capturing next frame
  delay(500); // 1 second delay


}



void followEdge() {

  
  // Capture a frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  uint8_t* image_data = fb->buf;

  
  int width = fb->width;
  int height = fb->height - 90; // your crop
  int third = width / 3;
  int halfY = height / 2;

  int left_count = 0;
  int mid_count = 0;
  int right_count = 0;
  
  int up_count = 0;
  int down_count = 0;
  // Analyze the image to find the line position
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uint8_t gray = image_data[y * width + x];

      // Thresholding
      if (gray > 180) {
        if (x < third) 
        left_count++;
        else if (x < 2 * third) 
        mid_count++;
        else 
        right_count++;
        if (y < halfY)
        up_count++;
        else
        down_count++;
      }
    }
  }

  // Normalize
  int total = left_count + mid_count + right_count + 1; // prevent divide by 0
  float left_ratio = (float)left_count / total;
  float right_ratio = (float)right_count / total;
  float mid_ratio = (float)mid_count / total;

  float totalY = up_count + down_count + 1;
  float up_ratio = (float)up_count / totalY;
  float down_ratio = (float)down_count / totalY;
  

  const int base_speed = 100;
  const int max_speed = 140;


  // Motor logic: more white on one side = turn toward that side
  int left_speed = base_speed + 1.5*(right_ratio - left_ratio) * base_speed;
  int right_speed = base_speed + 1.5*(left_ratio - right_ratio) * base_speed*0.94;

  if (left_ratio < 0.05 && right_ratio < 0.05) {
    // No line detected, stop or take corrective action
    left_speed = -100;
    right_speed = -100;
    Serial.println("No line detected, stopping.");
  }
  if (mid_ratio > 0.7)
  {
    // Line is mostly in the center, go straight
    left_speed = max_speed;
    right_speed = max_speed;
    Serial.println("Line detected in center, going straight.");
  }
  
  else if (left_ratio > 0.4 && right_ratio < 0.1) {
    // Sharp left turn
    left_speed = -100;
    right_speed = max_speed;
    Serial.println("Sharp Left Turn Detected!");
  }
  else if (right_ratio > 0.4 && left_ratio < 0.1) {
    // Sharp right turn
    left_speed = max_speed;
    right_speed = -100;
    Serial.println("Sharp Right Turn Detected!");
  }
  else if (up_ratio > 0.6 && down_ratio < 0.4) {
    // Line is mostly in the upper half, slow down
    left_speed *= 0.7;
    right_speed *= 0.7;
    Serial.println("Line detected in upper half, slowing down.");
  }
  else if (down_ratio > 0.6 && up_ratio < 0.4) {
    // Line is mostly in the lower half, speed up
    left_speed *= 1.3;
    right_speed *= 1.3;
    Serial.println("Line detected in lower half, speeding up.");
  }
  else
  {
    
  }
  
  
  // // Detect if we need to turn sharply (e.g., 90-degree turn)
  // if (left_ratio > 0.5) {
  //   // Line is on the left side, perform a sharp left turn
  //   // Stop right motor and continue turning with the left motor
  //   left_speed = -150;  // Stop left motor (or slow it down if you want a slower turn)
  //   right_speed = max_speed;  // Right motor should keep moving forward
  //   Serial.println("Sharp Left Turn Detected!");
  // }
  // else if (right_ratio > 0.5) {
  //   // Line is on the right side, perform a sharp right turn
  //   // Stop left motor and continue turning with the right motor
  //   left_speed = max_speed;  // Left motor should keep moving forward
  //   right_speed = -150;  // Stop right motor
  //   Serial.println("Sharp Right Turn Detected!");
  // }
  // else {
  //   // Otherwise, continue driving straight
  //   left_speed = max_speed;
  //   right_speed = max_speed;
  // }

  // Clamp
  left_speed = constrain(left_speed, -300, max_speed);
  right_speed = constrain(right_speed, -300, max_speed);


  // Apply motor speeds
  Serial.printf("Left: %d, Right: %d\n", left_speed, right_speed);
  
  
  setLeftPWM(left_speed);
  setRightPWM(right_speed);


  esp_camera_fb_return(fb);
  delay(20);
}
