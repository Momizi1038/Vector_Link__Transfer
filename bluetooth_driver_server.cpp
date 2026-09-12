#include "bluetooth_driver.h"
#include "btstack.h"
#include <string.h>

static uint8_t  rfcomm_channel_nr = 0;
static uint16_t rfcomm_cid        = 0;

static bool connected = false;
static bool can_send  = false;

/*
 * 最新データ保持用
 *
 * 古いデータをキューに積まない。
 * 常に最新のController Dataだけを保持する。
 */
static uint8_t tx_data[128];
static uint16_t tx_size = 0;
static bool send_pending = false;


bool bluetooth_is_connected(void)
{
    return connected;
}


bool bluetooth_can_send(void)
{
    return can_send;
}


/*
 * アプリケーション側から送信データを更新する。
 *
 * ここではrfcomm_send()を直接実行しない。
 * BTstackのCAN_SEND_NOWイベントを要求するだけ。
 */
int bluetooth_update_data(const uint8_t *data, uint16_t size)
{
    if(!connected) return -1;
    if(data == NULL || size == 0) return -2;
    if (size > sizeof(tx_data)) return -3;

    // 最新データに上書き
    memcpy(tx_data, data, size);
    tx_size = size;
    send_pending = true;

    // BTstackに「送信可能になったら通知してほしい」と要求
    rfcomm_request_can_send_now_event(rfcomm_cid);

    return 0;
}


/*
 * BTstackイベント処理
 */
void bluetooth_driver_server_handle_event(uint8_t packet_type, uint8_t *packet, uint16_t size){
    
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) return;

    bd_addr_t event_addr;

    switch (hci_event_packet_get_type(packet)) {
        /*
         * 接続要求
         */
        case RFCOMM_EVENT_INCOMING_CONNECTION:{
            rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);

            rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
            rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);

            printf("[RFCOMM] Incoming connection from %s (requested_channel=%d, cid=0x%04x)\n",
                bd_addr_to_str(event_addr), rfcomm_channel_nr, rfcomm_cid);

            int accept_result = rfcomm_accept_connection(rfcomm_cid);
            if (accept_result == 0){
                printf("[RFCOMM] Accepting connection (cid=0x%04x)...\n", rfcomm_cid);
            }else{
                printf("[RFCOMM] WARNING: rfcomm_accept_connection returned error %d\n", accept_result);
                rfcomm_cid = 0;
            }
            break;
        }
 
        // RFCOMM接続完了
        case RFCOMM_EVENT_CHANNEL_OPENED:{
            uint8_t status = rfcomm_event_channel_opened_get_status(packet);

            if(status != ERROR_CODE_SUCCESS){
                printf("[RFCOMM] ERROR: Channel open failed: 0x%02x\n", status);

                rfcomm_cid = 0;
                connected = false;
                can_send = false;

                break;
            }

            rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);

            connected = true;
            // まだCAN_SEND_NOWを受け取っていないのでfalse
            can_send = false;

            printf("[RFCOMM] ===== Channel opened! cid=0x%04x mtu=%d =====\n",
                rfcomm_cid, rfcomm_event_channel_opened_get_max_frame_size(packet));

            printf("[RFCOMM] [DATA TX START]\n");

            gap_discoverable_control(0);

            //  最初のCAN_SEND_NOWイベントを要求
            rfcomm_request_can_send_now_event(rfcomm_cid);
            break;
        }


        /*
         * RFCOMM送信可能イベント
         * ここが実際の送信トリガー
         */
        case RFCOMM_EVENT_CAN_SEND_NOW:{
            can_send = true;

            //  送信待ちデータがなければ終了
            if (!send_pending) break;

            if (!connected || rfcomm_cid == 0) break;

            //  最新データを送信
            int err = rfcomm_send(rfcomm_cid, tx_data, tx_size );

            if (err == 0) {
                // 送信完了
                send_pending = false;
                can_send = false;

                //  次回CAN_SEND_NOWを要求
                rfcomm_request_can_send_now_event(rfcomm_cid);
            }else if(err == BTSTACK_ACL_BUFFERS_FULL){

                /*
                 * ACLバッファが満杯。
                 *
                 * データは捨てずにsend_pendingを維持する。
                 */
                printf("[TX] Buffer full\n");
                can_send = false;

                /*
                 * 後で再び送信可能通知を要求
                 */
                rfcomm_request_can_send_now_event(rfcomm_cid);

            }else{
                printf("[TX] rfcomm_send error: %d\n", err);

                // 送信失敗なのでpendingは残す。
                can_send = false;
                rfcomm_request_can_send_now_event(rfcomm_cid);
            }

            break;
        }

        //RFCOMM切断
        case RFCOMM_EVENT_CHANNEL_CLOSED:
        {
            printf("[RFCOMM] Channel closed, returning to Discoverable mode\n");

            rfcomm_cid = 0;
            connected = false;
            can_send = false;

            send_pending = false;
            tx_size = 0;

            gap_discoverable_control(1);

            break;
        }


        default:
            break;
    }
}