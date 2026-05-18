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
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdlib.h"
#include "pico/rand.h"

#include "type.h"   // ds4_data 構造体

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define HEARTBEAT_PERIOD_MS   20    // 接続中の送信間隔 (ms) - Classic はCI制限なし
#define HEARTBEAT_IDLE_MS    500    // 未接続時のタイマー間隔 (ms)

// SPPチャンネル番号（1〜30、衝突しない任意の値）
#define SPP_RFCOMM_CHANNEL    1

// -------------------------------------------------------
// SDP レコード用バッファ
// -------------------------------------------------------
static uint8_t spp_service_buffer[150];

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
static uint8_t         rfcomm_channel_nr  = 0;
static uint16_t        rfcomm_cid         = 0;   // 0 = 未接続
static bool            connected          = false;
static bool            can_send           = false;  // rfcomm_grant_credits 後に送信可能

static ds4_data        controller_data;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t                 heartbeat;

// -------------------------------------------------------
// 前方宣言
// -------------------------------------------------------
static void make_romdom(ds4_data *output);
static void heartbeat_handler(struct btstack_timer_source *ts);
static void spp_packet_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size);

// -------------------------------------------------------
// ランダムデータ生成（テスト用）
// -------------------------------------------------------
static void make_romdom(ds4_data *output) {
    output->jyoutai = 134;
    output->L_x     = (uint8_t)(get_rand_32() & 0xFF);
    output->L_y     = (uint8_t)(get_rand_32() & 0xFF);
    output->R_x     = (uint8_t)(get_rand_32() & 0xFF);
    output->R_y     = (uint8_t)(get_rand_32() & 0xFF);
    output->L2      = (uint8_t)(get_rand_32() & 0xFF);
    output->R2      = (uint8_t)(get_rand_32() & 0xFF);
    output->key     = (uint8_t)(get_rand_32() & 0xFF);
    output->boton   = (uint8_t)(get_rand_32() & 0xFF);

    uint8_t sum = (uint8_t)(1 +
        output->jyoutai + output->L_x + output->L_y +
        output->R_x     + output->R_y + output->L2  +
        output->R2      + output->key + output->boton);
    output->checsam = sum % 255;
}

// -------------------------------------------------------
// heartbeatタイマーコールバック
// データ生成・RFCOMM送信
// -------------------------------------------------------
static void heartbeat_handler(struct btstack_timer_source *ts) {
    uint32_t next_interval;

    if (connected && can_send) {
        make_romdom(&controller_data);

        // RFCOMM送信（Classic SPP はストリーム型なので即座に送れる）
        int err = rfcomm_send(rfcomm_cid,
                              (uint8_t *)&controller_data,
                              sizeof(ds4_data));
        if (err == 0) {
            printf("[TX] %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
                   controller_data.L_x, controller_data.L_y,
                   controller_data.R_x, controller_data.R_y,
                   controller_data.L2,  controller_data.R2,
                   controller_data.key, controller_data.boton);
        } else if (err == BTSTACK_ACL_BUFFERS_FULL) {
            // バッファフル時は次回タイマーで再送
            printf("[TX] Buffer full, skip\n");
            can_send = false;
            rfcomm_request_can_send_now_event(rfcomm_cid);
        } else {
            printf("[TX] rfcomm_send error: %d\n", err);
        }
        next_interval = HEARTBEAT_PERIOD_MS;
    } else {
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
static void spp_packet_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_addr;

    switch (packet_type) {

        // --------------------------------------------------
        // HCI イベント
        // --------------------------------------------------
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
                    gap_local_bd_addr(event_addr);
                    printf("[SPP] BTstack up on %s\n", bd_addr_to_str(event_addr));
                    // Discoverable & Connectable に設定
                    gap_discoverable_control(1);
                    gap_connectable_control(1);
                    printf("[SPP] Waiting for connection...\n");
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // SSP非対応の相手のためにPINコードを返す（"0000"）
                    printf("[SPP] PIN code request from %s\n",
                           bd_addr_to_str(event_addr));
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // SSP: 数値比較を自動承認
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    gap_ssp_confirmation_response(event_addr);
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    // クライアントからの接続要求
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_cid        = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("[SPP] Incoming from %s channel=%d cid=0x%04x\n",
                           bd_addr_to_str(event_addr), rfcomm_channel_nr, rfcomm_cid);
                    rfcomm_accept_connection(rfcomm_cid);
                    break;

                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet) != ERROR_CODE_SUCCESS) {
                        printf("[SPP] Channel open failed: 0x%02x\n",
                               rfcomm_event_channel_opened_get_status(packet));
                        rfcomm_cid = 0;
                        break;
                    }
                    rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                    connected  = true;
                    can_send   = true;
                    printf("[SPP] Channel opened! cid=0x%04x mtu=%d\n",
                           rfcomm_cid,
                           rfcomm_event_channel_opened_get_max_frame_size(packet));
                    // 接続中は Discoverable を止めて不要な Inquiry 応答を減らす
                    gap_discoverable_control(0);
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    // バッファが空いたので次の heartbeat で送信できる
                    can_send = true;
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("[SPP] Channel closed\n");
                    rfcomm_cid = 0;
                    connected  = false;
                    can_send   = false;
                    // 再度 Discoverable にして次の接続を待つ
                    gap_discoverable_control(1);
                    break;

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
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    // --- BTstack プロトコルスタック初期化 ---
    l2cap_init();

    // RFCOMM 初期化
    rfcomm_init();
    rfcomm_register_service(spp_packet_handler, SPP_RFCOMM_CHANNEL, 0xFFFF);

    // SDP 初期化 & SPP サービスレコード登録
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, sdp_create_service_record_handle(),
                          SPP_RFCOMM_CHANNEL, "PicoW Controller");
    sdp_register_service(spp_service_buffer);
    printf("[SPP] SDP record registered\n");

    // デバイス名・クラス設定
    gap_set_local_name("PicoW Controller");
    // Device class: Toy (0x000804) — 用途に合わせて変更可
    gap_set_class_of_device(0x000804);
    // SSP (Secure Simple Pairing) 有効
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);

    // HCI イベントハンドラ登録
    hci_event_callback_registration.callback = &spp_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // heartbeat タイマー開始（初期は未接続間隔）
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_IDLE_MS);
    btstack_run_loop_add_timer(&heartbeat);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}