/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * [修正版 v2] BLE GATTサーバー - Picow Controller送信側
 *
 * 修正内容 (v2):
 *  5. hci_event_handler と att_packet_handler を分離
 *     → 同じ packet_handler を hci_add_event_handler と
 *       att_server_register_packet_handler の両方に登録すると
 *       HCI_EVENT_LE_META が2回処理され、ATTサーバー内部状態が乱れる
 *     → HCIイベント（接続・切断）は hci_event_handler のみで処理
 *     → ATTイベント（CAN_SEND_NOW）は att_packet_handler のみで処理
 */

#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/rand.h"

#include "type.h"
#include "ds4_data_gatt.h"

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define HEARTBEAT_PERIOD_MS    50    // 接続中のNotify送信間隔 (ms)
#define HEARTBEAT_IDLE_MS     500    // 未接続時のタイマー間隔 (ms)
                                     // 1msのままだとbtstack ランループを圧迫し
                                     // Central側のGATT探索が ATT 0x7F で失敗する

#define ADC_CHANNEL_TEMPSENSOR  4
#define APP_AD_FLAGS         0x06
#define DISCONECT_COUNT      1000    // 実験用切断カウンター（無効化は #if 0）

// -------------------------------------------------------
// アドバタイズデータ
// -------------------------------------------------------
static uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'P','i','c','o','W',' ','C','o','n','t','r','o','l','l','e','r',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
static int              le_notification_enabled = 0;
static hci_con_handle_t con_handle              = HCI_CON_HANDLE_INVALID;
static uint16_t         current_temp            = 0;

static ds4_data controller_data;
static int      Disconnect_counter = 0;
static bool     Send_notify        = true;
static bool     Send_data          = false;

// [修正5] HCIハンドラとATTハンドラで別々の登録構造体を使う
static btstack_packet_callback_registration_t hci_event_callback_registration;

static btstack_timer_source_t heartbeat;

extern uint8_t const profile_data[];

// -------------------------------------------------------
// 関数前方宣言
// -------------------------------------------------------
static void update_ds4_data(int stick);
static void make_romdom(ds4_data *output);
static void heartbeat_handler(struct btstack_timer_source *ts);

// -------------------------------------------------------
// heartbeatタイマーコールバック
// -------------------------------------------------------
static void heartbeat_handler(struct btstack_timer_source *ts) {
    uint32_t next_interval;

    if (le_notification_enabled) {
        make_romdom(&controller_data);
        Send_data = true;
        att_server_request_can_send_now_event(con_handle);
        next_interval = HEARTBEAT_PERIOD_MS;
    } else {
        // 未接続中は間隔を長くしてランループを圧迫しない
        next_interval = HEARTBEAT_IDLE_MS;
    }

    static bool led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    btstack_run_loop_set_timer(ts, next_interval);
    btstack_run_loop_add_timer(ts);
}

// -------------------------------------------------------
// [修正5] HCIイベントハンドラ（接続・切断・アドバタイズ設定のみ）
// hci_add_event_handler に登録する
// -------------------------------------------------------
static void hci_event_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {

        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("[BLE] BTstack up on %s\n", bd_addr_to_str(local_addr));

            {
                uint16_t  adv_int_min = 800;
                uint16_t  adv_int_max = 800;
                uint8_t   adv_type   = 0;
                bd_addr_t null_addr;
                memset(null_addr, 0, 6);
                gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type,
                                              0, null_addr, 0x07, 0x00);
                assert(adv_data_len <= 31);
                gap_advertisements_set_data(adv_data_len, (uint8_t *)adv_data);
                gap_advertisements_enable(1);
            }
            update_ds4_data(0);
            break;

        case HCI_EVENT_LE_META: {
            // [修正5] HCIハンドラのみで処理するためATTハンドラと重複しない
            uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
            if (subevent == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                uint16_t interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                printf("[BLE] Connected! handle=0x%04x interval=%.2fms\n",
                       con_handle, interval * 1.25f);
            }else if (subevent == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE) {
                uint8_t status = hci_subevent_le_connection_update_complete_get_status(packet);
                uint16_t interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
                printf("[BLE] Connection update: status=0x%02x interval=%d (%.2fms)\n",
               status, interval, interval * 1.25f);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("[BLE] Disconnected. reason=0x%02x\n", packet[5]);
            le_notification_enabled = 0;
            Send_data               = false;
            con_handle              = HCI_CON_HANDLE_INVALID;
            break;

        default:
            break;
    }
}

// -------------------------------------------------------
// [修正5] ATTパケットハンドラ（ATTイベントのみ）
// att_server_register_packet_handler に登録する
// -------------------------------------------------------
static void att_packet_handler(uint8_t packet_type, uint16_t channel,
                                uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {

        case ATT_EVENT_CAN_SEND_NOW:
#if 1
            // 実験用切断カウンター
            if (Disconnect_counter >= DISCONECT_COUNT) {
                printf("[EXP] Disconnect trigger\n");
                Disconnect_counter = 0;
                Send_notify        = false;
            } else {
                if (Send_data) Disconnect_counter++;
            }
#endif
            if (Send_notify && Send_data) {
                att_server_notify(con_handle,
                                  ATT_CHARACTERISTIC_CONTROLLER_DATA_01_VALUE_HANDLE,
                                  (uint8_t *)&controller_data,
                                  sizeof(ds4_data));
                printf("[TX] %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
                       controller_data.L_x, controller_data.L_y,
                       controller_data.R_x, controller_data.R_y,
                       controller_data.L2,  controller_data.R2,
                       controller_data.key, controller_data.boton);
                Send_data = false;
            }
            break;

        default:
            break;
    }
}

// -------------------------------------------------------
// ATT 読み取りコールバック
// -------------------------------------------------------
static uint16_t att_read_callback(hci_con_handle_t connection_handle,
                                   uint16_t att_handle, uint16_t offset,
                                   uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);
    if (att_handle == ATT_CHARACTERISTIC_CONTROLLER_DATA_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob(
            (const uint8_t *)&controller_data, sizeof(ds4_data),
            offset, buffer, buffer_size);
    }
    return 0;
}

// -------------------------------------------------------
// ATT 書き込みコールバック（Notify有効化）
// -------------------------------------------------------
static int att_write_callback(hci_con_handle_t connection_handle,
                               uint16_t att_handle, uint16_t transaction_mode,
                               uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);

    printf("[ATT] Write handle=0x%04x expected=0x%04x\n",
           att_handle,
           ATT_CHARACTERISTIC_CONTROLLER_DATA_01_CLIENT_CONFIGURATION_HANDLE);

    if (att_handle != ATT_CHARACTERISTIC_CONTROLLER_DATA_01_CLIENT_CONFIGURATION_HANDLE) {
        return 0;
    }

    le_notification_enabled =
        (little_endian_read_16(buffer, 0) ==
         GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);

    printf("[ATT] Notification %s\n", le_notification_enabled ? "ENABLED" : "DISABLED");

    con_handle = connection_handle;

    if (le_notification_enabled) {
        Send_notify = true;
        att_server_request_can_send_now_event(con_handle);
    }
    return 0;
}

// -------------------------------------------------------
// DS4データ初期化
// -------------------------------------------------------
static void update_ds4_data(int stick) {
    controller_data.jyoutai = 0x00;
    controller_data.L_x     = (uint8_t)stick;
    controller_data.L_y     = 128;
    controller_data.R_x     = 128;
    controller_data.R_y     = 128;
    controller_data.L2      = 0;
    controller_data.R2      = 0;
    controller_data.key     = 0x00;
    controller_data.boton   = 0x00;
    controller_data.checsam = 0x00;
    printf("[DS4] Data initialized. stick=%d\n", stick);
}

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
// キー入力コールバック
// -------------------------------------------------------
static volatile bool key_pressed = false;
static void key_pressed_func(void *param) {
    UNUSED(param);
    int key = getchar_timeout_us(0);
    if (key == 's' || key == 'S') {
        key_pressed = true;
    }
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(void) {
    stdio_init_all();

restart:
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    printf("[SYS] Press 'S' to stop Bluetooth\n");
    stdio_set_chars_available_callback(key_pressed_func, NULL);

    adc_init();
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    adc_set_temp_sensor_enabled(true);

    l2cap_init();
    sm_init();

    att_server_init(profile_data, att_read_callback, att_write_callback);

    // [修正5] HCIイベントとATTイベントを別々のハンドラに登録する
    //
    // 誤った登録方法（修正前）:
    //   hci_add_event_handler(&reg);            // packet_handler を登録
    //   att_server_register_packet_handler(packet_handler);  // 同じ関数を再登録
    //   → HCI_EVENT_LE_META が2回処理され Connected が2回表示・ATT状態が乱れる
    //
    // 正しい登録方法（修正後）:
    //   hci_add_event_handler   → hci_event_handler  (接続・切断・広告)
    //   att_server_register_*   → att_packet_handler (CAN_SEND_NOW のみ)
    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    att_server_register_packet_handler(att_packet_handler);

    // heartbeatタイマー開始（初期は未接続なのでIDLE間隔）
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_IDLE_MS);
    btstack_run_loop_add_timer(&heartbeat);

    hci_power_control(HCI_POWER_ON);

    key_pressed = false;
    while (!key_pressed) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(
            cyw43_arch_async_context(), at_the_end_of_time);
    }

    cyw43_arch_deinit();

    printf("[SYS] Press 'S' to restart Bluetooth\n");
    key_pressed = false;
    while (!key_pressed) {
        sleep_ms(1000);
    }
    goto restart;
    return 0;
}