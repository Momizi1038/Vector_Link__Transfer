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
#include "pico/multicore.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
//#include "pico/stdio_usb.h"  //TinyUSBと干渉するため無効化

#include "type.h"   // ds4_data 構造体
#include "bluetooth_driver.h"
#include "usb_driver.h"  // USB(TinyUSB)関連はここに隔離。tusb系ヘッダはここではincludeしない

#include "lib/E220Connect/e220.h"
#include "send_data.h"

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define HEARTBEAT_PERIOD_MS   1    // 接続中の送信間隔 (ms) - Classic はCI制限なし
#define HEARTBEAT_IDLE_MS     1    // 未接続時のタイマー間隔 (ms)

// SPPチャンネル番号（1〜30、衝突しない任意の値）
#define SPP_RFCOMM_CHANNEL    1
#define HCI_ENABLE_ROLE_SWITCH 0x0001

#define DEBUG_TX_LOG 1

#define ConectLED_D1 6
#define BlueLED_D2 3
#define Yellow_D3 2

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
// Input_dAta の実体は usb_driver.c 側に移動した

static btstack_packet_callback_registration_t hci_event_callback_registration;
//static btstack_timer_source_t                 heartbeat;

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
critical_section_t cs_ctrl_data;
critical_section_t cs_bt_connect;

ds4_data share_ctrl_data;
bool  share_bt_connect;

void core1_entry(){
    usb_driver_init();
    usb_ds4_color(50,200,250);
    while (true) {
        usb_driver_task();
        ds4_data d = usb_driver_get_data();

        critical_section_enter_blocking(&cs_ctrl_data);
        share_ctrl_data = d;
        critical_section_exit(&cs_ctrl_data);

        // critical_section_enter_blocking(&cs_bt_connect);
        // bool bt_ok = share_bt_connect;
        // critical_section_exit(&cs_bt_connect);

        if (Lora1_read_Aux()) {
            bool Lora_chack = Lora1_send_ds4(d,TARGET_CH); 
            gpio_put(ConectLED_D1, Lora_chack);
            #if DEBUG_TX_LOG
                uint16_t packet_seq = (static_cast<uint16_t>
                    (d.seq_H) << 8) | d.seq_L;
                printf("[SEQ]%d\n",packet_seq);
            #endif
        }
    }
}

int main(void) {
    // Phase 0: USB(TinyUSB)初期化 — 元のmain()と同じ呼び出し順序を維持
    //   board_init() → stdio_init_all() → (pio設定/tuh_configure/tusb_init/board_init_after_tusb)
    // という順序をusb_driver.h経由の2段階呼び出しで再現している。
    stdio_init_all();

    critical_section_init(&cs_ctrl_data);
    critical_section_init(&cs_bt_connect);

    multicore_launch_core1(core1_entry);

    // gpio_init(ConectLED_D1);
    // gpio_init(BlueLED_D2);
    // gpio_init(Yellow_D3);
    // gpio_set_dir(ConectLED_D1,GPIO_OUT);
    // gpio_set_dir(BlueLED_D2,GPIO_OUT);
    // gpio_set_dir(Yellow_D3,GPIO_OUT);
    // gpio_put(Yellow_D3,true);
    
    Lora1_init();


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
    printf("[SPP] SDP record registered (handle=0x%08lx, channel=%d)\n", 
           (unsigned long)service_handle, SPP_RFCOMM_CHANNEL);

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
    // heartbeat.process = &heartbeat_handler;
    // btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_IDLE_MS);
    // btstack_run_loop_add_timer(&heartbeat);
    // printf("[SPP] Heartbeat timer initialized\n");

    // ========== Phase 9: HCI 電源ON（最後！）==========
    printf("[SPP] Enabling Bluetooth...\n");
    hci_power_control(HCI_POWER_ON);
    //btstack_run_loop_execute();
    absolute_time_t nowTime = get_absolute_time();
    absolute_time_t next_send_bt = nowTime;
    //absolute_time_t last_log = 0;
    //uint32_t tx_count = 0;

    

    while(true){
          /*
        * Bluetooth接続状態を共有
        */
        bool connected_bt = bluetooth_is_connected();
        uint16_t last_sent_seq = 0;

        critical_section_enter_blocking(&cs_bt_connect);
        share_bt_connect = connected_bt;
        critical_section_exit(&cs_bt_connect);

        //Bluetooth接続中    
        if(connected_bt){
            //Core1から最新データを取得
            critical_section_enter_blocking(&cs_ctrl_data);
            controller_data = share_ctrl_data;
            critical_section_exit(&cs_ctrl_data);

            //SEQを確認 新しいController Dataが更新された場合のみ Bluetooth側へ渡す。       
            uint16_t now_seq = (static_cast<uint16_t>(controller_data.seq_H) << 8) | controller_data.seq_L;

            if(now_seq != last_sent_seq){
                // 最新データをBluetoothドライバへ渡すだけ。
                int err = bluetooth_update_data((uint8_t *)&controller_data,sizeof(ds4_data));
                if(err == 0){
                    last_sent_seq = now_seq;
                }

                #if DEBUG_TX_LOG
                    printf("[BT UPDATE] SEQ=%u ERR=%d\n", now_seq, err);
                #endif
            }   

            gpio_put(ConectLED_D1, true);

        }else{

            gpio_put(ConectLED_D1, false);

        /*
         * 再接続時にSEQ比較で送信されなくなることを防ぐ。
         */
            last_sent_seq = 0;
        }
        // LED 点滅
        static bool led_on = true;
        led_on = !led_on;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    }
    return 0;
}