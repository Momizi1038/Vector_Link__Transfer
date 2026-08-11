#ifndef USB_DRIVER_H
#define USB_DRIVER_H

// ここが最重要ポイント:
// tusb.h / bsp/board_api.h / pio_usb.h / hid_app.h は
// 絶対にこのヘッダからincludeしないこと。
// これを破ると server.cpp 経由で再び TinyUSB ヘッダが漏れ出し、
// btstack.h との hid_report_type_t 二重定義エラーが再発する。
#include "type.h"   // ds4_data 定義のみ利用

#ifdef __cplusplus
extern "C" {
#endif

ds4_data setDeta(uint8_t const* report, uint16_t len);

bool set_LED(int* output,uint8_t red,uint8_t green,uint8_t bure );

// ---- Phase 1: board_init() のみを行う ----
// 元の main() の呼び出し順を維持するため、
// 必ず stdio_init_all() より前に呼ぶこと。
void usb_driver_board_init(void);

// ---- Phase 2: TinyUSBホスト + PIO-USB の初期化 ----
// pio_usb設定 / tuh_configure / tusb_init / board_init_after_tusb を内包する。
// 必ず stdio_init_all() より後、cyw43_arch_init() より前に呼ぶこと
// （元のmain()の初期化順序と同一にするため）。
void usb_driver_init(void);

// ---- TinyUSBホストスタックのタスク処理 ----
// tuh_task() のラップ。USBデバイスのmount/report受信イベントは
// このタスクをどこかで周期的に呼ばない限り一切発火しないので注意。
// (heartbeat_handler内や専用のポーリングループから呼び出すことを想定)
void usb_driver_task(void);

void usb_ds4_color(uint8_t r,uint8_t g,uint8_t b);

// ---- 最新のUSBコントローラ入力データを取得 ----
ds4_data usb_driver_get_data(void);

#ifdef __cplusplus
}
#endif

#endif // USB_DRIVER_H