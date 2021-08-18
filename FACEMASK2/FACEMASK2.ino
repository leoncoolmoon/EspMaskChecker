#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
//#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"


#define NOMASKPIN 4
static box_array_t *net_boxes = NULL;

#define PWDN_GPIO_NUM     32
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


const char* ssid = "maskMonitor";
const char* password = "1234567890";
static boolean ota = false, wifi = false;
double Qc, Qd, Qe;
int Qb;
// td range for mask
const int diffmax = 50;
const int diffmin = 0;
// in the range means wearing mask
typedef struct
{
  size_t size;  //number of values used for filtering
  size_t index; //current value index
  size_t count; //value count
  int sum;
  int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;
static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 1;//ON/OFF THE FUNCTION

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

double updateQueue (double a, int f) {
  if (f < 0) {
    //reset
    Qb = 0;
    Qc = 0;
    Qd = 0;
    Qe = 0;
    return (0);
  } else {
    Qb = Qb + 1;
    Qc = Qd + (a - Qd) / Qb;
    Qe = Qe + (a - Qc) * (a - Qd);
    Qd = Qc;
    return (f == 0 ? Qd : Qb > 0 ? Qe / Qd : 0);
  }
}
static boolean checkmask_on_face(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id) {
  double avg, td;
  int  i, x0, y0, w0, h0, tx, ty, r, g, b;
  boolean mask = false;
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
    updateQueue(0, -1);
    // rectangle box
    x0 = (int)boxes->landmark[i].landmark_p[2];
    //L side of mouth/nose
    y0 = (int)boxes->landmark[i].landmark_p[5];
    //nose tip
    w0 = (int)boxes->landmark[i].landmark_p[8] - x0;
    //width of mouth/nose
    h0 = min((max((int)boxes->landmark[i].landmark_p[3] , (int)boxes->landmark[i].landmark_p[9]) - y0) * 2, (int)boxes->box[i].box_p[3]);
    //bottom of mouth, makesure cover the mouth but not bigger than face.
    for (int i = 0; i < w0 * h0; i++) {
      tx = x0 + i % w0;
      ty = y0 + i / w0 - 1;
      r = fb.data[(ty * fb.width + tx) * 3]; //b
      g = fb.data[(ty * fb.width + tx) * 3 + 1]; //g
      b = fb.data[(ty * fb.width + tx) * 3 + 2]; //r
      avg = (b + g) / 2;
      td = avg == 0 ? 0 : (r - avg) / avg;
      updateQueue(td, 0);
    }
    td = updateQueue(0, 1);
    Serial.print(td);
    mask = (td < diffmax)&& (td > diffmin);
    Serial.println(mask ? " >mask on" : " >mask off");
  }
  return mask;
}

static boolean detface() {
  int face_id = 0;
  bool detected = false;
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
    Serial.print(fb->width);
    Serial.print("x");
    Serial.print(fb->height);
    Serial.println("dl_matrix3du_alloc failed");
    esp_camera_fb_return(fb);
    fb = NULL;
    return false;
  } else {
    if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
      Serial.println("fmt2rgb888 failed");
      dl_matrix3du_free(image_matrix);
      esp_camera_fb_return(fb);
      fb = NULL;
      return false;
    } else {
      //ON/OFF THE FUNCTION
      if (detection_enabled) {
        net_boxes = face_detect(image_matrix, &mtmn_config);
      }
      if (net_boxes) {//FIND A FACE  TURE
        Serial.println("FIND face!");
        detected = true;
        detected =  detected && !checkmask_on_face(image_matrix, net_boxes, face_id);
      }
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }
  dl_matrix3du_free(image_matrix);
  return detected;
}



void detmask() {

 Serial.print(".");
  if (detface()) {
    //Serial.println("no mask");
    digitalWrite(NOMASKPIN, HIGH);
    delay(1000);

  }
  //Serial.println("mask on ");
  digitalWrite(NOMASKPIN, LOW);

};
void startFacedect() {
  pinMode(NOMASKPIN, OUTPUT);
  digitalWrite(NOMASKPIN, LOW);
  ra_filter_init(&ra_filter, 20);
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  Serial.println("");
  Serial.println("face dection configed");
};
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(NOMASKPIN, INPUT);
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }


  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
  wifi = digitalRead(NOMASKPIN) == HIGH;
  if (wifi) {
    //AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.println("");
    Serial.print("WiFi started ssid: ");
    Serial.println(ssid);
    Serial.print("password: ");
    Serial.println(password);
    Serial.print("Camera Ready! Use 'http://192.168.4.1");
    Serial.println("' to connect");

    ArduinoOTA.setHostname(ssid);
    ArduinoOTA.setPassword(password);
    ArduinoOTA
    .onStart([]() {
      String type;
      ota = true;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      ota = false;
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
  } else {
    Serial.println("");
    Serial.println("WiFi disabled");
  }

  startFacedect();
}

void loop() {
  if (!ota) {
    /*  usually when people wearing medical mask their face either not be able to detected 
     *  or the colour difference around nose and mouse is too small  
     */
    detmask();
    delay(2000);
  }
  // put your main code here, to run repeatedly:
  if (wifi) {
    ArduinoOTA.handle();
  }
}
