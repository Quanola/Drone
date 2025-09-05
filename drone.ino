//Quan Nguyen Drone

#include "esp_camera.h"
#include "fd_forward.h"    
#include "fr_forward.h"  
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"


#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#define TXD2 17
#define RXD2 16

// Image parameters
#define FRAME_SIZE FRAMESIZE_QVGA  // 320x240
#define FACE_CENTER_TOLERANCE 20   // pixels

camera_fb_t * fb = NULL;
mtmn_config_t mtmn_config = {0};

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART2


  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAME_SIZE;
  config.fb_count = 2;

  // Init camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true);
  }


  mtmn_config = mtmn_init_config();
  Serial.println("Setup complete");
}

void loop() {
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  box_array_t *faces = face_detect(fb, &mtmn_config);

  if (faces && faces->len > 0) {

    box_t *face = &faces->box[0];
    int face_x = (face->box_p[0] + face->box_p[2]) / 2;
    int face_y = (face->box_p[1] + face->box_p[3]) / 2;

    int center_x = fb->width / 2;
    int center_y = fb->height / 2;

    int error_x = face_x - center_x;
    int error_y = face_y - center_y;

    Serial.printf("Face Center: (%d, %d), Error: (%d, %d)\n", face_x, face_y, error_x, error_y);

    Serial2.printf("<X:%d,Y:%d>\n", error_x, error_y);
  }

  esp_camera_fb_return(fb);
  delay(100); // ~10 FPS
}
