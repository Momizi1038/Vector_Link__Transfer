#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 送信側 (server) が使う関数 ----
void bluetooth_driver_server_init(void);
bool bluetooth_is_connected(void);
bool bluetooth_can_send(void);
int  bluetooth_send(const uint8_t *data, uint16_t size);

// server.cpp の spp_packet_handler から呼ぶイベント振り分け関数
void bluetooth_driver_server_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size);

// ---- 受信側 (client) が使う関数 ----
void bluetooth_driver_client_init(void);
bool bluetooth_client_is_connected(void);
void bluetooth_driver_client_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size);
uint16_t bluetooth_client_get_cid(void);  // rfcomm_grant_credits で使用

#ifdef __cplusplus
}
#endif