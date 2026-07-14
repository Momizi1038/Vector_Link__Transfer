/**
 * Bluetooth Classic SPP サーバー - Picow Controller送信側
 *
 * BLE (GATT/Notify) から Bluetooth Classic (SPP/RFCOMM) への移行版
 *
 * 変更点:
 *  - BLEアドバタイズ → Classic Inquiry応答 (gap_set_local_name / gap_set_class_of_device)
 *  - GATT ATTサーバー → l2cap_init + rfcomm_init + sdp_init
 *  - att_server_notify → rfcomm_send
 *  - Connection Interval概念の削除（RFCOMM はストリーム型）
 *  - heartbeat間隔を固定 HEARTBEAT_PERIOD_MS に統一（動的調整不要）
 *  - ds4_data_gatt.h / profile_data 不要
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/rand.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
//#include "pico/stdio_usb.h"  //TinyUSBと干渉するため無効化

#include "type.h"   // ds4_data 構造体
#include "bluetooth_driver.h"

#include "bsp/board_api.h"
#include "tusb.h"
#include "hid_app.h"
#include "pio_usb.h"
#include "send_data.h"

#include "hardware/uart.h"

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define HEARTBEAT_PERIOD_MS   1    // 接続中の送信間隔 (ms) - Classic はCI制限なし
#define HEARTBEAT_IDLE_MS    500    // 未接続時のタイマー間隔 (ms)

// SPPチャンネル番号（1〜30、衝突しない任意の値）
#define SPP_RFCOMM_CHANNEL    1
#define HCI_ENABLE_ROLE_SWITCH 0x0001

#define DEBUG_TX_LOG 1

//PIO USB Config
#define CFG_TUH_RPI_PIO_USB 1

// -------------------------------------------------------
// SDP レコード用バッファ
// -------------------------------------------------------
static uint8_t spp_service_buffer[512];

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
// static uint8_t         rfcomm_channel_nr  = 0;
// static uint16_t        rfcomm_cid         = 0;   // 0 = 未接続
// static bool            connected          = false;
// static bool            can_send           = false;  // rfcomm_grant_credits 後に送信可能

static ds4_data        controller_data;
ds4_data Input_dAta;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t                 heartbeat;

// -------------------------------------------------------
// 前方宣言
// -------------------------------------------------------
static ds4_data make_romdom(void);
static void heartbeat_handler(struct btstack_timer_source *ts);

// -------------------------------------------------------
// ランダムデータ生成（テスト用）
// -------------------------------------------------------
static ds4_data make_romdom(void) {
    ds4_data output;

    output.jyoutai = 134;
    output.L_x     = (uint8_t)(get_rand_32() & 0xFF);
    output.L_y     = (uint8_t)(get_rand_32() & 0xFF);
    output.R_x     = (uint8_t)(get_rand_32() & 0xFF);
    output.R_y     = (uint8_t)(get_rand_32() & 0xFF);
    output.L2      = (uint8_t)(get_rand_32() & 0xFF);
    output.R2      = (uint8_t)(get_rand_32() & 0xFF);
    output.key     = (uint8_t)(get_rand_32() & 0xFF);
    output.boton   = (uint8_t)(get_rand_32() & 0xFF);

    uint8_t sum = (uint8_t)(1 +
        output.jyoutai + output.L_x + output.L_y +
        output.R_x     + output.R_y + output.L2  +
        output.R2      + output.key + output.boton);
    output.checsam = sum % 256;

    return output;
}

static ds4_data get_ps4data_by_usb(void) {
  tuh_task();
  return Input_dAta;
}

// -------------------------------------------------------
// heartbeatタイマーコールバック
// データ生成・RFCOMM送信
// -------------------------------------------------------
static void heartbeat_handler(struct btstack_timer_source *ts) {
    uint32_t next_interval;
    
    controller_data = make_romdom();
    if(bluetooth_send((uint8_t *)&controller_data,sizeof(ds4_data)) == 0){
        #if DEBUG_TX_LOG
        printf("[TX] %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
               controller_data.L_x, controller_data.L_y,
               controller_data.R_x, controller_data.R_y,
               controller_data.L2, controller_data.R2,
               controller_data.key, controller_data.boton);
        #endif
        next_interval = HEARTBEAT_PERIOD_MS;
    }else{
        next_interval = HEARTBEAT_IDLE_MS;
    }

    // LED 点滅
    static bool led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    btstack_run_loop_set_timer(ts, next_interval);
    btstack_run_loop_add_timer(ts);
}

// -------------------------------------------------------
// SPP / RFCOMM / HCI パケットハンドラ
// Classic では1つのハンドラで HCI・RFCOMM イベントを処理できる
// -------------------------------------------------------
static void spp_packet_handler(uint8_t packet_type, uint16_t channel,uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_addr;

    bluetooth_driver_server_handle_event(packet_type, packet, size);

    if(packet_type != HCI_EVENT_PACKET) return;
    switch (packet_type) {
        // --------------------------------------------------
        // HCI イベント
        // --------------------------------------------------
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case BTSTACK_EVENT_STATE:{
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
                    gap_local_bd_addr(event_addr);
                    printf("[SPP] BTstack up on %s\n", bd_addr_to_str(event_addr));
                    // Discoverable & Connectable に設定
                    gap_discoverable_control(1);
                    gap_connectable_control(1);
                    printf("[SPP] Waiting for connection...\n");
                    break;
                }

                case HCI_EVENT_PIN_CODE_REQUEST:{
                    // SSP非対応の相手のためにPINコードを返す（"0000"）
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    printf("[SPP] PIN code request from %s\n", bd_addr_to_str(event_addr));
                    gap_pin_code_response(event_addr, "0000");
                    break;
                }

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:{
                    // SSP: 数値比較を自動承認
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    gap_ssp_confirmation_response(event_addr);
                    break;
                }

                case HCI_EVENT_CONNECTION_COMPLETE:{
                    uint8_t status = hci_event_connection_complete_get_status(packet);
                    if (status != ERROR_CODE_SUCCESS) {
                        printf("[HCI] Connection failed: 0x%02x\n", status);
                        break;
                    }
                    hci_event_connection_complete_get_bd_addr(packet, event_addr);
                    printf("[HCI] ACL Connection established from %s\n", 
                    bd_addr_to_str(event_addr));
                    break;
                }
                default:
                    break;
            }
            break;

        // --------------------------------------------------
        // RFCOMM データ受信（サーバーは基本送信のみだが念のため処理）
        // --------------------------------------------------
        case RFCOMM_DATA_PACKET:
            printf("[RX] %d bytes received\n", size);
            break;

        default:
            break;
    }
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(void) {
    board_init();
    stdio_init_all();

    bool chack = false;
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = 4;// 例: D+ピン(GPIO27)
    pio_cfg.pinout = PIO_USB_PINOUT_DPDM; // DM=DP-1
    chack = tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    printf("tuh_configure:%d\n", chack);

    chack = tusb_init(1);
    printf("tusb_init: %d\n", chack);

    board_init_after_tusb();
    // if (board_init_after_tusb) {
    //     board_init_after_tusb();
    // }

    // for (int i = 0; i < 30; i++) {
    //     if (stdio_usb_connected()) break;
    //     sleep_ms(100);
    // }

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    // ========== Phase 1: HCI イベント登録（最初！）==========
    hci_event_callback_registration.callback = &spp_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    printf("[SPP] HCI event handler registered\n");

    // ========== Phase 2: プロトコルスタック初期化 ==========
    l2cap_init();
    printf("[SPP] L2CAP initialized\n");

    rfcomm_init();
    printf("[SPP] RFCOMM initialized\n");

    // ========== Phase 3: RFCOMM リスナー登録（正規 MTU） ==========
    int service_err = rfcomm_register_service(spp_packet_handler, SPP_RFCOMM_CHANNEL, 672);
    if (service_err != 0) {
        printf("[SPP] ERROR: rfcomm_register_service failed: %d\n", service_err);
    } else {
        printf("[SPP] RFCOMM service registered on channel %d (MTU=672)\n", SPP_RFCOMM_CHANNEL);
    }

    sdp_init();
    printf("[SPP] SDP initialized\n");

    // ========== Phase 4: SDP レコード登録 ==========
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    uint32_t service_handle = sdp_create_service_record_handle();
    spp_create_sdp_record(spp_service_buffer, service_handle,
                          SPP_RFCOMM_CHANNEL, "PicoW Controller");
    sdp_register_service(spp_service_buffer);
    printf("[SPP] SDP record registered (handle=0x%08x, channel=%d)\n", 
           service_handle, SPP_RFCOMM_CHANNEL);

    // ========== Phase 5: Inquiry/GAP 設定 ==========
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    gap_set_local_name("PicoW Controller");
    gap_set_class_of_device(0x000100);
    printf("[SPP] Device class set to 0x000100 (Miscellaneous Device)\n");

    // ========== Phase 6: SSP/セキュリティ設定 ==========
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_NO_BONDING);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    printf("[SPP] SSP configured (auto-accept mode)\n");

    // ========== Phase 7: Discoverable/Connectable 明示的設定 ==========
    gap_discoverable_control(1);
    gap_connectable_control(1);
    printf("[SPP] Discoverable & Connectable mode enabled\n");

    // ========== Phase 8: heartbeat タイマー設定 ==========
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_IDLE_MS);
    btstack_run_loop_add_timer(&heartbeat);
    printf("[SPP] Heartbeat timer initialized\n");

    // ========== Phase 9: HCI 電源ON（最後！）==========
    printf("[SPP] Enabling Bluetooth...\n");
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}