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

    board_init_after_tusb();
}

void usb_driver_task(void) {
    tuh_task();
}

ds4_data usb_driver_get_data(void) {
    return Input_dAta;
}