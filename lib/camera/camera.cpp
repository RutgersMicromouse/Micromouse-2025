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
  delay(50); // 1 second delay


}

void followEdge() {
  // Capture a frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  uint8_t* image_data = fb->buf;
  const uint8_t threshold = 128;  // Adjust based on lighting

  // ---- Intersection Detection (blob analysis) ----
  int rows[] = { fb->height / 4, fb->height / 2, 3 * fb->height / 4 };
  int row_blob_counts[3] = {0};

  for (int r = 0; r < 3; r++) {
    int y = rows[r];
    bool in_blob = false;
    for (int x = 0; x < fb->width; x++) {
      uint8_t gray = image_data[y * fb->width + x];
      if (gray > threshold) {
        if (!in_blob) {
          row_blob_counts[r]++;
          in_blob = true;
        }
      } else {
        in_blob = false;
      }
    }
  }

  int total_blobs = row_blob_counts[0] + row_blob_counts[1] + row_blob_counts[2];
  Serial.printf("Blobs: top=%d, mid=%d, bot=%d, total=%d\n",
                row_blob_counts[0], row_blob_counts[1], row_blob_counts[2], total_blobs);

  // ---- Intersection decision logic ----
  if (total_blobs >= 3 || row_blob_counts[1] >= 2) {
    Serial.println("T-intersection or crossroad detected");
    // Add logic here to choose direction (left/right/forward) based on path planning
  } else if (row_blob_counts[0] >= 1 && row_blob_counts[2] >= 1) {
    Serial.println("Diagonal or complex intersection detected");
  } else if (row_blob_counts[2] == 1 && row_blob_counts[1] == 0) {
    Serial.println("Dead end detected");
  } else {
    // ---- Normal line following ----
    int edge_position = -1;
    int y = fb->height / 2; // Middle row
    for (int x = 0; x < fb->width; x++) {
      uint8_t gray = image_data[y * fb->width + x];
      if (gray > threshold) {
        edge_position = x;
        break;
      }
    }

    if (edge_position != -1) {
      int error = (fb->width / 2) - edge_position;
      int motor_speed = 100 + error; // Simple proportional control
      Serial.printf("Following line - Edge position: %d, Error: %d, Motor speed: %d\n",
                    edge_position, error, motor_speed);

      // Add your motor control here:
      setLeftPWM(motor_speed);
      setRightPWM(-motor_speed);
    } else {
      Serial.println("Line not found");
      // Stop or reverse slightly, depending on your bot's logic
    }
  }

  esp_camera_fb_return(fb);
  delay(100);
}
