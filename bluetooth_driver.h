#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 送信側 (server) が使う関数 ----
void bluetooth_driver_server_init(void);
bool bluetooth_is_connected(void);
bool bluetooth_can_send(void);
int  bluetooth_send(const uint8_t *data, uint16_t size);

#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

bool bluetooth_is_connected(void);
bool bluetooth_can_send(void);

/*
 * 最新の送信データを更新する。
 * 実際のRFCOMM送信はCAN_SEND_NOWイベントで行う。
 */
int bluetooth_update_data(const uint8_t *data, uint16_t size);

void bluetooth_driver_server_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size);

#endif

// server.cpp の spp_packet_handler から呼ぶイベント振り分け関数
// void bluetooth_driver_server_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size);

// ---- 受信側 (client) が使う関数 ----
void bluetooth_driver_client_init(void);
bool bluetooth_client_is_connected(void);
void bluetooth_driver_client_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size);
uint16_t bluetooth_client_get_cid(void);  // rfcomm_grant_credits で使用

#ifdef __cplusplus
}
#endif