// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
#define NOMASKPIN 4
static box_array_t *net_boxes = NULL;
typedef struct {
  size_t size; //number of values used for filtering
  size_t index; //current value index
  size_t count; //value count
  int sum;
  int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 1;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};
static boolean visualize = false;
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

static int ra_filter_run(ra_filter_t * filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str) {
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...) {
  char loc_buf[64];
  char * temp = loc_buf;
  int len;
  va_list arg;
  va_list copy;
  va_start(arg, format);
  va_copy(copy, arg);
  len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
  va_end(copy);
  if (len >= sizeof(loc_buf)) {
    temp = (char*)malloc(len + 1);
    if (temp == NULL) {
      return 0;
    }
  }
  vsnprintf(temp, len + 1, format, arg);
  va_end(arg);
  rgb_print(image_matrix, color, temp);
  if (len > 64) {
    free(temp);
  }
  return len;
}
double Qc, Qd, Qe;
int Qb;
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
static boolean draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id, boolean disp) {
  int x, y, w, h, i, color_index;
  boolean mask = false;
  uint32_t color[] = {FACE_COLOR_YELLOW, FACE_COLOR_RED, FACE_COLOR_GREEN, FACE_COLOR_BLUE, FACE_COLOR_CYAN, FACE_COLOR_PURPLE}; //m1n2m4
  if (disp){
    color_index = face_id <= 0?0:2 ;   
    }

  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
    // rectangle box
    x = (int)boxes->box[i].box_p[0];
    y = (int)boxes->box[i].box_p[1];
    w = (int)boxes->box[i].box_p[2] - x + 1;
    h = (int)boxes->box[i].box_p[3] - y + 1;
    if(disp){
    fb_gfx_drawFastHLine(&fb, x, y, w, color[color_index]);
    fb_gfx_drawFastHLine(&fb, x, y + h - 1, w, color[color_index]);
    fb_gfx_drawFastVLine(&fb, x, y, h, color[color_index]);
    fb_gfx_drawFastVLine(&fb, x + w - 1, y, h, color[color_index]);
    }
#if 0        // landmark
    int j, x0, y0;
    for (j = 0; j < 10; j += 2) {
      x0 = (int)boxes->landmark[i].landmark_p[j];
      y0 = (int)boxes->landmark[i].landmark_p[j + 1];
      // uint32_t color[] = {FACE_COLOR_YELLOW, FACE_COLOR_RED, FACE_COLOR_GREEN, FACE_COLOR_BLUE, FACE_COLOR_CYAN, FACE_COLOR_PURPLE}; //m1(x2y3)n2(x4y5)m4(x8y9)
      fb_gfx_fillRect(&fb, x0, y0, 3, 3, color[(int)j / 2]);

    }
#endif
#if 0
    //export rgb888 into text
    int a;
    Serial.print("lenth=");
    Serial.println(fb.width * fb.height * 3);
    for (a = 0; a < fb.width * fb.height * 3; a++) {

      Serial.print(fb.data[a]);

      if (a % 20) {
        Serial.print(",");
      } else {
        Serial.println(",");
      }
    }

#endif
#if 1
    //mask / mouth area
    int x0, y0;
    int w0, h0, tx, ty, r, g, b,diff = -300;
    double avg, td;
    updateQueue(0, -1);
    x0 = (int)boxes->landmark[i].landmark_p[2];
    //L side of mouth/nose
    y0 = (int)boxes->landmark[i].landmark_p[5];
    //nose tip
    w0 = (int)boxes->landmark[i].landmark_p[8] - x0;
    //width of mouth/nose
    h0 = min((max((int)boxes->landmark[i].landmark_p[3] , (int)boxes->landmark[i].landmark_p[9]) - y0) * 2,(int)boxes->box[i].box_p[3]);
   //bottom of mouth, makesure cover the mouth but not bigger than face.
    if(disp){
    color_index = 1;
    fb_gfx_drawFastHLine(&fb, x0, y0, w0, color[color_index]);
    fb_gfx_drawFastHLine(&fb, x0, y0 + h0 - 1, w0, color[color_index]);
    fb_gfx_drawFastVLine(&fb, x0, y0, h0, color[color_index]);
    fb_gfx_drawFastVLine(&fb, x0 + w0 - 1, y0, h0, color[color_index]);
    }
    for (int i = 0; i < w0 * h0; i++) {
      tx = x0 + i % w0;
      ty = y0 + i / w0 - 1;
      r = fb.data[(ty * fb.width + tx) * 3]; //b
      g = fb.data[(ty * fb.width + tx) * 3 + 1]; //g
      b = fb.data[(ty * fb.width + tx) * 3 + 2]; //r
      /*  Serial.print("rgb=[");
        Serial.print(r);
        Serial.print(",");
        Serial.print(g);
        Serial.print(",");
        Serial.print(b);
        Serial.println("]");
      */
      avg = (b + g) / 2;
      td = avg == 0 ? 0 : (r - avg) / avg;
      // Serial.print("td= ");
      // Serial.println(td);
      updateQueue(td, 0);
    }
    td = updateQueue(0, 1);
    Serial.print(td);
    mask = (td < diff);
    Serial.println(mask ? " >mask on" : " >mask off");
    if(disp){
       rgb_printf(image_matrix, mask ?FACE_COLOR_GREEN:FACE_COLOR_RED, mask ? " >mask on %u" : " >mask off %u" ,td);
       }
#endif
  }
  return mask;
}


static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}


static boolean detface() {
  int face_id = 0;
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  bool detected = false;
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
   if (detection_enabled) {
        net_boxes = face_detect(image_matrix, &mtmn_config);
      }

      if (net_boxes || fb->format != PIXFORMAT_JPEG) {
        if (net_boxes) {
          detected = true;
          detected = detected && !  draw_face_boxes(image_matrix, net_boxes, face_id, false);
         
        }


      }
      esp_camera_fb_return(fb);
      fb = NULL;
    }

  }
  dl_matrix3du_free(image_matrix);
  return detected;
}




static esp_err_t capture_handler(httpd_req_t *req) {
  visualize = true;
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t out_len, out_width, out_height;
  uint8_t * out_buf;
  bool s;
  bool detected = false;
  int face_id = 0;
  if (!detection_enabled || fb->width > 400) {
    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = {req, 0};
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    visualize = false;
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
  }

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
    esp_camera_fb_return(fb);
    Serial.print(fb->width);
    Serial.print("x");
    Serial.print(fb->height);
    Serial.println("dl_matrix3du_alloc failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;

  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if (!s) {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

     if (detection_enabled) {
        net_boxes = face_detect(image_matrix, &mtmn_config);
      }

  if (net_boxes) {
    detected = true;
    draw_face_boxes(image_matrix, net_boxes, face_id,true);
 
  }

  jpg_chunking_t jchunk = {req, 0};
  s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
  dl_matrix3du_free(image_matrix);
  if (!s) {
    Serial.println("JPEG compression failed");
    return ESP_FAIL;
  }

  int64_t fr_end = esp_timer_get_time();
  Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  visualize = true;
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  bool detected = false;
  int face_id = 0;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    detected = false;
    face_id = 0;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      if (!detection_enabled || fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      } else {

        image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

        if (!image_matrix) {
          Serial.print(fb->width);
          Serial.print("x");
          Serial.print(fb->height);
          Serial.println("dl_matrix3du_alloc failed");
          res = ESP_FAIL;
        } else {
          if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
            Serial.println("fmt2rgb888 failed");
            res = ESP_FAIL;
          } else {
            fr_ready = esp_timer_get_time();
        if (detection_enabled) {
              net_boxes = face_detect(image_matrix, &mtmn_config);
            }
            fr_face = esp_timer_get_time();
            fr_recognize = fr_face;
            if (net_boxes || fb->format != PIXFORMAT_JPEG) {
              if (net_boxes) {
                detected = true;

                draw_face_boxes(image_matrix, net_boxes, face_id,true);
               
              }
              if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)) {
                Serial.println("fmt2jpg failed");
                res = ESP_FAIL;
              }
              esp_camera_fb_return(fb);
              fb = NULL;
              visualize = false;
            } else {
              _jpg_buf = fb->buf;
              _jpg_buf_len = fb->len;
            }
            fr_encode = esp_timer_get_time();
          }
          dl_matrix3du_free(image_matrix);
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t ready_time = (fr_ready - fr_start) / 1000;
    int64_t face_time = (fr_face - fr_ready) / 1000;
    int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    int64_t process_time = (fr_encode - fr_start) / 1000;

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    /*Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
                  (uint32_t)(_jpg_buf_len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                  avg_frame_time, 1000.0 / avg_frame_time,
                  (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
                  (detected) ? "DETECTED " : "", face_id
                 );*/
  }

  last_frame = 0;
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t * s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  } else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
  else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
  else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
  else if (!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
  else if (!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
  else if (!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
  else if (!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
  else if (!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
  else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
  else if (!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
  else if (!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
  else if (!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
  else if (!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
  else if (!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
  else if (!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
  else if (!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
  else if (!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
  else if (!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
  else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
  else if (!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
  else if (!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
  else if (!strcmp(variable, "face_detect")) {
    detection_enabled = val;
  } else {
    res = -1;
  }

  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t * s = esp_camera_sensor_get();
  char * p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
  p += sprintf(p, "\"face_detect\":%u,", detection_enabled);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  sensor_t * s = esp_camera_sensor_get();
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}
void detmask() {
  if (!visualize) {
    Serial.println("detecting ...");
    if (detface()) {
      Serial.println("no mask");
      digitalWrite(NOMASKPIN, HIGH);
      delay(1000);

    }
    Serial.println("mask on ");
    digitalWrite(NOMASKPIN, LOW);
  }
}

void startFacedect() {
  pinMode(NOMASKPIN, OUTPUT);
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

}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t status_uri = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t capture_uri = {
    .uri       = "/capture",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
