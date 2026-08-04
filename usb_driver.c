/**
 * usb_driver.c
 *
 * TinyUSB (Host) + PIO-USB 関連の初期化・タスク処理を
 * server.cpp から完全に隔離するための抽象化レイヤー。
 *
 * server.cpp は btstack.h を include するため、
 * TinyUSB系ヘッダ (tusb.h / bsp/board_api.h / pio_usb.h) を
 * 同じ翻訳単位でincludeすると hid_report_type_t が
 * BTstack側 (btstack_hid.h) と二重定義されコンパイルエラーになる。
 * このファイルだけがTinyUSB系ヘッダを見える状態にし、
 * server.cpp からは usb_driver.h 経由のプレーンな関数呼び出しのみで
 * USB処理を行えるようにする。
 */

#include "usb_driver.h"

#include <stdio.h>
#include "bsp/board_api.h"
#include "tusb.h"
#include "pio_usb.h"
#include "hid_app.h"
#include "type.h"

ds4_data setDeta(uint8_t const* report, uint16_t len){
  (void)len;
  const uint8_t dpad_str[] = {0b00000001,0b00000011,0b00000010,0b00000110,0b00000100,0b00001100,0b00001000,0b00001001,0b00000000};
  uint8_t pad_deta_bit = 0b00000000;

  ds4_data data = {0}; 
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
  data.checsam = data.checsam ;

  return data;
}

 bool set_LED(int* output,uint8_t red,uint8_t green,uint8_t bure ){
  output[0] = 0b00001000;
  output[1] = red;
  output[2] = green;
  output[3] = bure;
  output[4] = (output[0] + output[1] + output[2] + output[3])%255;
  return true;
 }

// hid_app.c 側では `extern ds4_data Input_dAta;` として参照される実体。
// 従来 server.cpp にあった定義をこちらに移動した。
ds4_data Input_dAta;

void usb_driver_board_init(void) {
    board_init();
}

void usb_driver_init(void) {
    bool chack = false;

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = 4;                    // 例: D+ピン(GPIO27)
    pio_cfg.pinout = PIO_USB_PINOUT_DPDM;  // DM = DP-1

    chack = tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    printf("tuh_configure:%d\n", chack);

    chack = tusb_init(1);
    printf("tusb_init: %d\n", chack);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
}

void usb_driver_task(void) {
    tuh_task();
}

void usb_ds4_color(uint8_t r,uint8_t g,uint8_t b){
    hid_app_task(r, g, b);
}

ds4_data usb_driver_get_data(void) {
    //tuh_task();
    return Input_dAta;
}