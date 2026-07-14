#include "hid_app.h"
#include "send_data.h"

ds4_data setDeta(uint8_t const* report, uint16_t len){
  (void)len;
  const uint8_t dpad_str[] = {0b00000001,0b00000011,0b00000010,0b00000110,0b00000100,0b00001100,0b00001000,0b00001001,0b00000000};
  uint8_t pad_deta_bit = 0b00000000;

  ds4_data data{}; 
  static sony_ds4_report_t privous_data = {};
  static int counter_disconect = 0;

  uint8_t const report_id = report[0];
  report++;
  len--;

  if(report_id == 1){
    sony_ds4_report_t ds4_report;
    memcpy(&ds4_report, report, sizeof(ds4_report));
    privous_data.counter = ds4_report.counter;

    data.L_x = ds4_report.x;
    data.L_y = ds4_report.y;
    data.R_x = ds4_report.z;
    data.R_y = ds4_report.rz;

    data.L2 = ds4_report.l2_trigger;
    data.R2 = ds4_report.r2_trigger;

    if(ds4_report.triangle) pad_deta_bit |= 0b10000000;
    if(ds4_report.circle)   pad_deta_bit |= 0b01000000;
    if(ds4_report.cross)    pad_deta_bit |= 0b00100000;
    if(ds4_report.square)   pad_deta_bit |= 0b00010000;
    pad_deta_bit |= dpad_str[ds4_report.dpad];

    data.key = pad_deta_bit;
    pad_deta_bit = 0b00000000;

    if(ds4_report.l1)     pad_deta_bit |= 0b10000000;
    if(ds4_report.r1)     pad_deta_bit |= 0b01000000;
    if(ds4_report.l3)     pad_deta_bit |= 0b00100000;
    if(ds4_report.r3)     pad_deta_bit |= 0b00010000;
    if(ds4_report.share)  pad_deta_bit |= 0b00001000;
    if(ds4_report.option) pad_deta_bit |= 0b00000100;
    if(ds4_report.ps)     pad_deta_bit |= 0b00000010;
    if(ds4_report.tpad)   pad_deta_bit |= 0b00000001;
    
    data.boton = pad_deta_bit;

    data.jyoutai |= 0b10000010;//固有設定

    if(ds4_report.ps && ds4_report.share){
      data.jyoutai |= 0b00000001;
    }
    
    if(diff_report(&privous_data, &ds4_report)){
      counter_disconect = 0;
      privous_data = ds4_report;
    }else{
      counter_disconect = counter_disconect + 1;
    }

    if(counter_disconect <= 500){
      data.jyoutai |= 0b00000100;
    }else{
      data.jyoutai |= 0b00000000;
    }
    
  }else{
    data.jyoutai |= 0b10000111;
  }

  int sum = 0;
  sum = data.boton + data.jyoutai + data.key + data.L2 + data.L_x + data.L_y + data.R2 + data.R_x + data.R_y;
  data.checsam = sum % 255;
  data.checsam = data.checsam + 1;

  return data;
}

bool changeData(int* output , ds4_data rewdata){
  output[0] = int(rewdata.jyoutai);
  output[1] = int(rewdata.L_x);
  output[2] = int(rewdata.L_y);
  output[3] = int(rewdata.R_x);
  output[4] = int(rewdata.R_y);
  output[5] = int(rewdata.L2);
  output[6] = int(rewdata.R2);
  output[7] = int(rewdata.key);
  output[8] = int(rewdata.boton);
  output[9] = int(rewdata.checsam);
  return true;
 }

 bool set_LED(int* output,uint8_t red,uint8_t green,uint8_t bure ){
  output[0] = 0b00001000;
  output[1] = red;
  output[2] = green;
  output[3] = bure;
  output[4] = (output[0] + output[1] + output[2] + output[3])%255;
  return true;
 }