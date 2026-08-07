#include "bluetooth_driver.h"
#include "btstack.h"

static uint8_t         rfcomm_channel_nr  = 0;
static uint16_t        rfcomm_cid         = 0;   // 0 = 未接続
static bool            connected          = false;
static bool            can_send           = false;  // rfcomm_grant_credits 後に送信可能

bool bluetooth_is_connected(void) { return connected; }
bool bluetooth_can_send(void)     { return can_send; }

int bluetooth_send(const uint8_t *data, uint16_t size) {

    printf("can_snd=%d",can_send);

    if (!connected || !can_send) return -1;
    can_send = false; // 送信前にfalseに

    // RFCOMM送信（Classic SPP はストリーム型なので即座に送れる）
    int err = rfcomm_send(rfcomm_cid, (uint8_t*)data, size);

    if (err == 0){
        rfcomm_request_can_send_now_event(rfcomm_cid);
    }else if (err == BTSTACK_ACL_BUFFERS_FULL){
        // バッファフル時は次回タイマーで再送
        printf("[TX] Buffer full, skip\n");
        can_send = false;
        rfcomm_request_can_send_now_event(rfcomm_cid);
    }else{
        can_send = true;
        printf("[TX] rfcomm_send error: %d\n", err);
    }

    printf("con=%d cid=%04x\n",connected,rfcomm_cid);
    return err;
}

void bluetooth_driver_server_handle_event(uint8_t packet_type,uint8_t *packet, uint16_t size) {
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    bd_addr_t event_addr;

    switch (hci_event_packet_get_type(packet)) {
        case RFCOMM_EVENT_INCOMING_CONNECTION:{
            // クライアントからの接続要求
            rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
            rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
            rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
            printf("[RFCOMM] Incoming connection from %s (requested_channel=%d, cid=0x%04x)\n",
                   bd_addr_to_str(event_addr), rfcomm_channel_nr, rfcomm_cid);

            // 接続受け入れ
            int accept_result = rfcomm_accept_connection(rfcomm_cid);
            if (accept_result == 0){
                printf("[RFCOMM] Accepting connection (cid=0x%04x)...\n", rfcomm_cid);
            }else{
                printf("[RFCOMM] WARNING: rfcomm_accept_connection returned error %d\n", accept_result);
                rfcomm_cid = 0;
            }
            break;
        }

        case RFCOMM_EVENT_CHANNEL_OPENED:{
            if (rfcomm_event_channel_opened_get_status(packet) != ERROR_CODE_SUCCESS){
                printf("[RFCOMM] ERROR: Channel open failed: 0x%02x\n",
                       rfcomm_event_channel_opened_get_status(packet));
                rfcomm_cid = 0;
                break;
            }
            rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
            connected = true;
            can_send = true;
            printf("[RFCOMM] ===== Channel opened! cid=0x%04x mtu=%d =====",
                   rfcomm_cid,
                   rfcomm_event_channel_opened_get_max_frame_size(packet));
            printf(" [DATA TX START]\n");
            // 接続中は Discoverable を止めて不要な Inquiry 応答を減らす
            gap_discoverable_control(0);
            break;
        }

        case RFCOMM_EVENT_CAN_SEND_NOW:{
            // バッファが空いたので次の heartbeat で送信できる
            can_send = true;
            break;
        }

        case RFCOMM_EVENT_CHANNEL_CLOSED:{
            printf("[RFCOMM] Channel closed, returning to Discoverable mode\n");
            rfcomm_cid = 0;
            connected = false;
            can_send = false;
            // 再度 Discoverable にして次の接続を待つ
            gap_discoverable_control(1);
            break;
        }

        default:
            break;
        }
}