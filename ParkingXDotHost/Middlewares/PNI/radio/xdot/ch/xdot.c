#include <stdint.h>
#include <string.h>

#include "radio.h"
#include "xdot.h"
#include "xdot_hex.h"
#include "xdot_at_cmd.h"
#include "xdot_at_resp.h"
#include "parking_buffer.h"
#include "ble_clock.h"
#include "main.h"
#include "stm32l0xx_hal_gpio.h"

static UART_HandleTypeDef *m_xdot_huart;

static char m_xdot_buff_tx[PNI_RADIO_BUFFER_TX_LEN] = { 0 };

#define XDOT_BUFF_W_PTR(h) ( \
        (PNI_RADIO_BUFFER_RX_LEN - h->hdmarx->Instance->CNDTR) \
        & (PNI_RADIO_BUFFER_RX_LEN - 1))

static uint8_t m_xdot_buff_rx[PNI_RADIO_BUFFER_RX_LEN] = { 0 };
static uint8_t m_xdot_buff_rx_idx = 0;

static uint8_t m_xdot_resp_data[PNI_RADIO_BUFFER_RX_LEN] = { 0 };
static uint8_t m_xdot_resp_data_idx = 0;
static uint32_t m_xdot_resp_hash = XDOT_AT_RESP_HASH_INIT;

static const int XDOT_SF_DR_MAP[XDOT_ISM_BAND_MAX][XDOT_SF_MAX] = {
    [XDOT_ISM_BAND_EU868] = {
        [XDOT_SF_12_BW_125] = XDOT_DATA_RATE_DR0,
        [XDOT_SF_11_BW_125] = XDOT_DATA_RATE_DR1,
        [XDOT_SF_10_BW_125] = XDOT_DATA_RATE_DR2,
        [XDOT_SF_9_BW_125] = XDOT_DATA_RATE_DR3,
        [XDOT_SF_8_BW_125] = XDOT_DATA_RATE_DR4,
        [XDOT_SF_8_BW_500] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_7_BW_125] = XDOT_DATA_RATE_DR5,
        [XDOT_SF_7_BW_250] = XDOT_DATA_RATE_DR6,
        [XDOT_SF_FSK] = XDOT_DATA_RATE_DR7,
    },
    [XDOT_ISM_BAND_US915] = {
        [XDOT_SF_12_BW_125] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_11_BW_125] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_10_BW_125] = XDOT_DATA_RATE_DR0,
        [XDOT_SF_9_BW_125] = XDOT_DATA_RATE_DR1,
        [XDOT_SF_8_BW_125] = XDOT_DATA_RATE_DR2,
        [XDOT_SF_8_BW_500] = XDOT_DATA_RATE_DR4,
        [XDOT_SF_7_BW_125] = XDOT_DATA_RATE_DR3,
        [XDOT_SF_7_BW_250] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_FSK] = XDOT_DATA_RATE_INVALID,
    },
    [XDOT_ISM_BAND_CH470] = {
        [XDOT_SF_12_BW_125] = XDOT_DATA_RATE_DR0,
        [XDOT_SF_11_BW_125] = XDOT_DATA_RATE_DR1,
        [XDOT_SF_10_BW_125] = XDOT_DATA_RATE_DR2,
        [XDOT_SF_9_BW_125] = XDOT_DATA_RATE_DR3,
        [XDOT_SF_8_BW_125] = XDOT_DATA_RATE_DR4,
        [XDOT_SF_8_BW_500] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_7_BW_125] = XDOT_DATA_RATE_DR5,
        [XDOT_SF_7_BW_250] = XDOT_DATA_RATE_INVALID,
        [XDOT_SF_FSK] = XDOT_DATA_RATE_INVALID,
    },
};

static const int XDOT_DR_PAYLOAD_MAP[XDOT_ISM_BAND_MAX][XDOT_DATA_RATE_MAX] = {
    [XDOT_ISM_BAND_EU868] = {
        [XDOT_DATA_RATE_DR0] = 51,
        [XDOT_DATA_RATE_DR1] = 51,
        [XDOT_DATA_RATE_DR2] = 51,
        [XDOT_DATA_RATE_DR3] = 115,
        [XDOT_DATA_RATE_DR4] = 242,
        [XDOT_DATA_RATE_DR5] = 242,
        [XDOT_DATA_RATE_DR6] = 242,
        [XDOT_DATA_RATE_DR7] = 50,
    },
    [XDOT_ISM_BAND_US915] = {
        [XDOT_DATA_RATE_DR0] = 11,
        [XDOT_DATA_RATE_DR1] = 53,
        [XDOT_DATA_RATE_DR2] = 129,
        [XDOT_DATA_RATE_DR3] = 242,
        [XDOT_DATA_RATE_DR4] = 242,
        [XDOT_DATA_RATE_DR5] = 0,
        [XDOT_DATA_RATE_DR6] = 0,
        [XDOT_DATA_RATE_DR7] = 0,
    },
    [XDOT_ISM_BAND_CH470] = {
        [XDOT_DATA_RATE_DR0] = 51,
        [XDOT_DATA_RATE_DR1] = 51,
        [XDOT_DATA_RATE_DR2] = 51,
        [XDOT_DATA_RATE_DR3] = 115,
        [XDOT_DATA_RATE_DR4] = 222,
        [XDOT_DATA_RATE_DR5] = 222,
        [XDOT_DATA_RATE_DR6] = 0,
        [XDOT_DATA_RATE_DR7] = 0,
    },
};

static Xdot_State m_xdot_state = {
    .flags = XDOT_STATE_FLAG_NONE,
    .join_retry_count = 0,
    .tx_retry_count = 0,
};

static Xdot_AtCmdIface m_xdot_at_cmd_iface = { 0 };

static Node_Info LoRaNode;
static Node_Info *LoRaNodestr = &LoRaNode;

 int xdot_wake_mcu(Lora_WAKE_T Wake);
static void xdot_node_sta_check(Node_Info *LoRa_temp);

#if 0
static void xdot_wake(void)
{
    HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port, XDOT_WAKE_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
   // HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port, XDOT_WAKE_Pin, GPIO_PIN_RESET);
    //HAL_Delay(10);
}
#endif

#if 0
static int xdot_buffer_to_int(uint8_t *buffer, uint16_t count)
{
    int i;
    int ret = 0;

    for (i = 0; i < count; i++) {
        ret = (ret << 3) + (ret << 1) + (buffer[i]) - '0';
    }
    return ret;
}
#endif

static uint8_t xdot_buff_rx_get(void)
{
    uint8_t c = 0; uint8_t a,b;

    if (m_xdot_buff_rx_idx != XDOT_BUFF_W_PTR(m_xdot_huart)) {
        c = m_xdot_buff_rx[m_xdot_buff_rx_idx++];
        m_xdot_buff_rx_idx &= (PNI_RADIO_BUFFER_RX_LEN - 1);
    //PNI_PRINTF("szhang add m_xdot_buff_rx_idx=%d,CNDTR=%d,c=%d\r\n",m_xdot_buff_rx_idx,m_xdot_huart->hdmarx->Instance->CNDTR,c);
	}
	    return c;
}

static int xdot_get_resp1(Xdot_AtResp *resp)
{
    uint8_t b;
    int resp_id = 0;

   //while((huart->Instance->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);
  // 	{
  //   return PNI_RADIO_RET_EBUSY;
//	 }
   //  return PNI_RADIO_RET_OK;
}

static int xdot_get_resp(Xdot_AtResp *resp)
{
    uint8_t b;
    int resp_id = 0;
    while (b = xdot_buff_rx_get()) {
        if (b == 0x0D) {
            // ignore CR
            continue;
        } else if (b == 0x0A) {
            // end command on LF
            resp_id = xdot_at_resp_hash_search(m_xdot_resp_hash);
			PNI_PRINTF("szhang add resp_id=%d\r\n",resp_id);
            switch (resp_id) {

            case XDOT_AT_RESP_OK:
                resp->ret = PNI_RADIO_RET_OK;
                m_xdot_state.flags &= ~(XDOT_STATE_FLAG_RESP_WAIT);
                goto exit_done;

            case XDOT_AT_RESP_ERROR:
                resp->ret = PNI_RADIO_RET_EINVAL;
                m_xdot_state.flags &= ~(XDOT_STATE_FLAG_RESP_WAIT);
                goto exit_done;

            default:
                if (m_xdot_resp_data_idx) {
                    resp->count = m_xdot_resp_data_idx;
                    resp->hash = m_xdot_resp_hash;
                    memcpy(resp->data, m_xdot_resp_data, resp->count);
                }
                break;
            }
            m_xdot_resp_data_idx = 0;
            m_xdot_resp_hash = XDOT_AT_RESP_HASH_INIT;

        } else {
            m_xdot_resp_data[m_xdot_resp_data_idx++] = b;
            m_xdot_resp_data_idx %= PNI_RADIO_BUFFER_RX_LEN;
            m_xdot_resp_hash = xdot_at_resp_hash_char(b, m_xdot_resp_hash);
        }
    }

    return PNI_RADIO_RET_EBUSY;

exit_done:
    m_xdot_resp_data_idx = 0;
    m_xdot_resp_hash = 0;
    return resp->ret;
}

static int xdot_resp_wait(Xdot_AtResp *resp, int retries)
{
    int ret ;

    while (retries--) {
        ret = xdot_get_resp(resp);
        if (ret != PNI_RADIO_RET_EBUSY) {
            XDOT_D("ret: %d, retries: %d", ret, retries);
            return ret;
        }
        Clock_Wait(1);
    }
    return PNI_RADIO_RET_EIO;
}

static int xdot_at_cmd_exec(char *buffer, uint16_t count)
{
    /* if sleeping, wake it up */
    if (m_xdot_state.flags & XDOT_STATE_FLAG_IN_SLEEP) {
        XDOT_I("waking up ...");
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_IN_SLEEP);
        xdot_wake_mcu(MODULE_RESUME);

    }
    // already waiting for a resp, return wait time
    if (m_xdot_state.flags & XDOT_STATE_FLAG_RESP_WAIT) {
        return PNI_RADIO_RET_EBUSY;
    }

    XDOT_D("cmd: %s", buffer);
    // set waiting for resp flag
    m_xdot_state.flags |= XDOT_STATE_FLAG_RESP_WAIT;
    if (HAL_UART_Transmit(m_xdot_huart, (uint8_t *)buffer, count,
            PNI_RADIO_UART_TIMEOUT) != HAL_OK) {
        XDOT_E();
        return PNI_RADIO_RET_EIO;
    }
    return PNI_RADIO_RET_OK;
}

static int xdot_get_sf_dr(Xdot_IsmBand ism_band, Xdot_SpreadingFactor sf)
{
    return XDOT_SF_DR_MAP[ism_band][sf];
}

static int xdot_set_tx_data_rate(Xdot_IsmBand ism_band, Xdot_SpreadingFactor sf)
{
    int dr = xdot_get_sf_dr(ism_band, sf);

    if (dr < 0) {
        return dr;
    }

    return xdot_at_cmd_set_data_rate(&m_xdot_at_cmd_iface, dr);
}

static int xdot_get_tx_max_payload(Xdot_IsmBand ism_band,
        Xdot_SpreadingFactor sf)
{
    int dr = xdot_get_sf_dr(ism_band, sf);

    if (dr < 0) {
        return dr;
    }

    return XDOT_DR_PAYLOAD_MAP[ism_band][dr];
}

static int xdot_sleep(PNI_RadioSleepMode mode)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    /*
     * This may seem redundant but will be helpful in the future if we
     * use different hardware with different sleep modes.
     */
    switch (mode) {
        case PNI_RADIO_SLEEP_MODE_STANDBY:
            ret = xdot_wake_mcu(MODULE_RESUME);
        case PNI_RADIO_SLEEP_MODE_STOP:
            ret = xdot_wake_mcu(MODULE_SUSPEND);
    }

    if (ret != PNI_RADIO_RET_OK) {
        return ret;
    }
  // XDOT_I("going to sleep ...");
XDOT_I("szhang add going to sleep ...");
   m_xdot_state.flags |= XDOT_STATE_FLAG_IN_SLEEP;

    return PNI_RADIO_RET_OK;
}


#if 0
static int xdot_get_tx_next(void)
{
    Xdot_AtResp resp = { 0 };
    int ret = 0;

    if ((ret = xdot_at_cmd_get_tx_next(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    return xdot_buffer_to_int(resp.data, resp.count);
}
#endif

#if 0
static int xdot_get_network_join_status(Xdot_NetworkJoinStatus *status)
{
    Xdot_AtResp resp = { 0 };
    int ret = 0;

    *status = XDOT_NET_JOIN_STATUS_NOT_JOINED;

    if ((ret = xdot_at_cmd_get_net_join_status(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    if (resp.data[0] == '1') {
        *status = XDOT_NET_JOIN_STATUS_JOINED;
    }
    return PNI_RADIO_RET_OK;
}
#endif

#if 0
static int xdot_set_pre_join_settings(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // set network id
    if ((ret = xdot_at_cmd_set_network_id(&m_xdot_at_cmd_iface,
            cfg->ni_type, cfg->network_id)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 100))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // set network key
    if ((ret = xdot_at_cmd_set_network_key(&m_xdot_at_cmd_iface,
            cfg->nk_type, cfg->network_key)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 100))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    return PNI_RADIO_RET_OK;
}
#endif

static int xdot_set_post_join_settings(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // set TXDR
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_set_tx_data_rate(cfg->ism_band, cfg->sf))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    // wait for resp
    if ((ret = xdot_resp_wait(&resp, 200))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_mode_mcu(Lora_Mode_T WorkMode)
{
  int rval=PNI_RADIO_RET_OK;

  if(WorkMode == MODE_CMD)
  {
    //command mode
    HAL_GPIO_WritePin(XDOT_MODE_GPIO_Port,XDOT_MODE_Pin, GPIO_PIN_SET);
    Clock_Wait(50);
  }
  else if(WorkMode == MODE_PASSTHROUGH)
  {
    //Passthrought mode
    HAL_GPIO_WritePin(XDOT_MODE_GPIO_Port,XDOT_MODE_Pin, GPIO_PIN_RESET);
    Clock_Wait(50);
  }
  else
  {rval=PNI_RADIO_RET_EINVAL;}

  return rval;
}


static int xdot_join_if_needed(Xdot_Cfg *cfg)
{
    Xdot_NetworkJoinStatus status = XDOT_NET_JOIN_STATUS_NOT_JOINED;
    int ret;

    // get join status

    xdot_node_sta_check(LoRaNodestr);
    if(LoRaNode.Join == 0)
    {status = XDOT_NET_JOIN_STATUS_NOT_JOINED;}
    else
    {status = XDOT_NET_JOIN_STATUS_JOINED;}

    // already joined, return ok
    if (status == XDOT_NET_JOIN_STATUS_JOINED) {
        m_xdot_state.join_retry_count = 0;
        return PNI_RADIO_RET_OK;
    }

    // check for retry threshold
    if (m_xdot_state.join_retry_count++ > XDOT_JOIN_RETRY_THRD) {
        return m_xdot_state.join_retry_count * XDOT_JOIN_RETRY_MULT_MS;
    }
    m_xdot_state.flags |= XDOT_STATE_FLAG_JOIN_WAIT;
    return XDOT_TX_RESP_DELAY_MIN_MS;
}

static int xdot_send_hex(uint8_t *buffer, uint16_t size)
{
    char buff[XDOT_SENDB_MAX_SIZE + 1];
    uint16_t count = 0;
    int i = 0;
    int j=0;
    for (i = 0; i < size; i++) {
        count += snprintf(buff + count, sizeof(buff) - count, "%02x", buffer[i]);
    }

    return xdot_at_cmd_sendb(&m_xdot_at_cmd_iface, buff);
}

static int xdot_send_check_tx_resp(void)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // check for resp
   ret = xdot_get_resp1(&resp);
    // if resp recv, clear flag
    if (ret != PNI_RADIO_RET_EBUSY) {
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_SEND_WAIT);

        // if send ok, return ok
        if (ret == PNI_RADIO_RET_OK) {
            m_xdot_state.tx_retry_count = 0;
            return ret;
        }

        // else ret send delay
        if (m_xdot_state.tx_retry_count++ > XDOT_TX_RETRY_THRD) {
            return m_xdot_state.tx_retry_count * XDOT_TX_RETRY_MULT_MS;
        }
    }

    return ret;
}

#if 0
static int xdot_send_check_join_resp(Xdot_Cfg *cfg)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    // check for resp
    ret = xdot_get_resp(&resp);

    // if resp recv, clear flag
    if (ret != PNI_RADIO_RET_EBUSY) {
        m_xdot_state.flags &= ~(XDOT_STATE_FLAG_JOIN_WAIT);
    }

    // if join failed, return default delay
    if (ret != PNI_RADIO_RET_OK) {
        return XDOT_TX_RESP_DELAY_MIN_MS;
    }

    // join success, apply settings
    return xdot_set_post_join_settings(cfg);
}
#endif
static int xdot_send_data(void *self, uint8_t *buffer, uint16_t size)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    int ret;
    int i =0;

    if(LIERDA_WAKE_PIN_STATE != GPIO_PIN_SET)// if in sleep mode wake up it
    {
    HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port,XDOT_WAKE_Pin, GPIO_PIN_SET);
    Clock_Wait(500);
    }
     ret = xdot_join_if_needed(cfg);
     if (ret) {
     return ret;

     }

	while(LIERDA_BUSY_PIN_STATE==GPIO_PIN_RESET)// 判断是不是处于空闲状态
		{
		 Clock_Wait(500);
		 return PNI_RADIO_RET_EBUSY;
		}
		ret= HAL_UART_Transmit(m_xdot_huart, (uint8_t *)buffer,size ,PNI_RADIO_UART_TIMEOUT);


	      return ret;
}

static int xdot_send(void *self, PNI_ParkingBufferIface *parking_buffer)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    uint16_t packet_size = sizeof(parking_buffer->buffer.data[0]);
    uint16_t packet_count = parking_buffer->len(parking_buffer);
    uint16_t payload_size = 0;
    uint16_t payload_size_max = 0; /* depends on TXDR */
    uint16_t payload_packet_count = 0;
    uint16_t payload_packet_max = 0; /* depends on payload_size_max */
    PNI_ParkingPacket data[PNI_PARKING_BUFFER_COUNT] = { 0 };
    int index = 0;
    int ret = 0;

    /* get max payload size */
    ret = xdot_get_tx_max_payload(cfg->ism_band, cfg->sf);
    if (ret < 1) {
        return ret;
    }
    payload_size_max = ret;

    /* maximum number of packets per payload */
    payload_packet_max = (payload_size_max / packet_size);

    /* number of packets to send */
    payload_packet_count = MIN(packet_count, payload_packet_max);

    /* check if already in send */
    if (m_xdot_state.tx_packet_count) {
        /* if caculated packet count is less than tx packet count then
         * something is out of sync. need to figure out how to handle this
         */
        if (payload_packet_count < m_xdot_state.tx_packet_count) {
            XDOT_E("bad packet count");
        }
        payload_packet_count = m_xdot_state.tx_packet_count;
    } else {
        m_xdot_state.tx_packet_count = payload_packet_count;
    }

    /* copy packets from queue into buffer */
    for (index = 0; index < payload_packet_count; index++) {
        parking_buffer->get(parking_buffer, &data[index], index);

    }

    /* size of data to send */
    payload_size = payload_packet_count * packet_size;

    /* if ok, return the number of packets sent */

    ret = xdot_send_data(iface, (uint8_t *)data, payload_size);// 发送数据通过passthrough mode
    if (ret == PNI_RADIO_RET_OK) {
        /* reset tracked count */
        parking_buffer->adv_out(parking_buffer, payload_packet_count);
        m_xdot_state.tx_packet_count = 0;
    }

    /* ok, error, busy, tx wait (>=1000) */
    return ret;
}

/////////////////=======need modify
static int xdot_recv(void *self, uint8_t *buffer, uint16_t size)
{
    /* TODO:
     *
     * - Make sure receive buffer (size) is big enough to hold a UART response
     *   (PNI_RADIO_BUFFER_RX_LEN).
     * - Make sure receive buffer is big enough to hold max AT+RECV packet
     *   (242:string), (242:hex -- possible typo in AT reference?)
     */
    XDOT_I("szhang add xdot_recv======================\r\n");
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = iface->params;
    Xdot_AtResp resp = { 0 };
    int ret = PNI_RADIO_RET_OK;

    if (LIERDA_WORK_MODE == GPIO_PIN_SET)  // Lierda module don't used  AT command  in passthrough mode

    {
     if ((ret = xdot_at_cmd_recv(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK) {

     return ret;

     }

     }

      else
      {
      if (HAL_UART_Transmit(m_xdot_huart, (uint8_t *)buffer, size,PNI_RADIO_UART_TIMEOUT) != HAL_OK) {

      XDOT_E();
      return PNI_RADIO_RET_EIO;

      }

      return PNI_RADIO_RET_OK;

      }

      if ((ret = xdot_resp_wait(&resp, XDOT_RESP_DELAY_MS)) != PNI_RADIO_RET_OK) {
      return ret;
      }

    /*
     * if echo is enabled and there is no data from the server the respsonse
     * will contain the AT+RECV command, we should ignore it and return ok.
     */
    if (xdot_at_resp_hash_search(resp.hash) == XDOT_AT_RESP_AT_CMD_RECV) {
        return PNI_RADIO_RET_OK;
    }

    if (cfg->rx_type == XDOT_RX_OUT_TYPE_HEX) {
        /* Hex mode returns hex string -- convert into bytes */
        return xdot_hex_to_bytes(resp.data, resp.count, buffer, size);
    } else if (cfg->rx_type == XDOT_RX_OUT_TYPE_RAW) {
        /* Raw mode returns raw bytes */
        memcpy(buffer, &resp.data, resp.count);
        return resp.count;
    } else {
        /* Invalid RX mode */
        return PNI_RADIO_RET_EINVAL;
    }
}


static int xdot_get_device_id(PNI_RadioDeviceId *id)
{
    Xdot_AtResp resp = { 0 };
    int parts[PNI_RADIO_DEVICE_ID_SIZE];
    int i;
    int ret;

    if ((ret = xdot_at_cmd_get_deveui(&m_xdot_at_cmd_iface))
            != PNI_RADIO_RET_OK) {
        return ret;
    }

    if ((ret = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
        return ret;
    }

    if (PNI_RADIO_DEVICE_ID_SIZE != sscanf((const char *)resp.data,
            "%X-%X-%X-%X-%X-%X-%X-%X",
            &parts[0], &parts[1], &parts[2], &parts[3],
            &parts[4], &parts[5], &parts[6], &parts[7])) {
        return PNI_RADIO_RET_EINVAL;
    }

    for (i = 0; i < PNI_RADIO_DEVICE_ID_SIZE; i++) {
        id->bytes[i] = (uint8_t)parts[i];
    }

    return PNI_RADIO_RET_OK;
}

static int xdot_busy_status_check(void)
{
  int rval=0;
  if(HAL_GPIO_ReadPin(XDOT_BUSY_GPIO_Port,XDOT_BUSY_Pin))
  {//LORA MODULE IN IDLE STATUS.
    rval=MODULE_IDLE;
    //XDOT_D("Busy gpio H!!!");
  }
  else
  {//LORA MODULE IN busy STATUS.
    rval=MODULE_BUSY;
    //XDOT_D("Busy gpio L!!!");

  }
  return rval;
}

static int xdot_stat_status_check(void)
{
  int rval=0;
  if(HAL_GPIO_ReadPin(XDOT_STAT_GPIO_Port,XDOT_STAT_Pin))
  {//lora's internet is normal.
    rval=NET_OK;
    //XDOT_D("Stat gpio H!!!");
  }
  else
  {//lora's internet is abnormal.
    rval=NET_ERROR;

    //XDOT_D("Stat gpio L!!!");
  }
  return rval;
}

static int xdot_reset_mcu(void)
{
    /* toggle XDOT_NRST */
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);//>=1ms
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(200);//>=150ms
    return PNI_RADIO_RET_OK;
}

int xdot_wake_mcu(Lora_WAKE_T Wake)
{
  int rval=PNI_RADIO_RET_OK;
   GPIO_InitTypeDef GPIO_InitStructure;
  if(Wake == MODULE_RESUME)
  {//wake mode
    if(HAL_GPIO_ReadPin(XDOT_WAKE_GPIO_Port,XDOT_WAKE_Pin) != GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port,XDOT_WAKE_Pin, GPIO_PIN_SET);
      Clock_Wait(50);
    }
  }
  else if(Wake == MODULE_SUSPEND)
  {//sleep mode

    if(HAL_GPIO_ReadPin(XDOT_WAKE_GPIO_Port,XDOT_WAKE_Pin) != GPIO_PIN_RESET)
    {
      HAL_GPIO_WritePin(XDOT_WAKE_GPIO_Port,XDOT_WAKE_Pin, GPIO_PIN_RESET);

      Clock_Wait(50);
    }

  }
  else
  {rval=PNI_RADIO_RET_EINVAL;}
  return rval;
}


static char *xdot_string_str(char *str, char *dest)
{
#define STR_BUFF_LEN	0x100
  int i = STR_BUFF_LEN;
  char *cp = str;
  char *s1, *s2;

  if (*dest == 0)
  {
    return str;
  }

  while(i--)
  {
    s1 = cp;
    s2 = dest;

    while((*s1 == *s2) && *s1 && *s2)
    {
      s1++;
      s2++;
    }
    if(!*s2)
      return cp;
    cp++;
  }

  return NULL;
}

static int xdot_get_node_info(Node_Info *lora_temp)
{
  char *ptr = NULL;
  uint8_t temp1=0,temp2=0;
  uint8_t index=0;
  int rval = PNI_RADIO_RET_OK;
  Xdot_AtResp resp = { 0 };

  xdot_mode_mcu(MODE_CMD);

  xdot_wake_mcu(MODULE_RESUME);

  Clock_Wait(50);

  // deveui get & wait for resp
  if ((rval = xdot_at_cmd_get_deveui(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("deveui get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get deveui Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+DEVEUI:")) != NULL)
  {
		for(uint8_t i=0;i<8;i++)
		{
			temp1 = *((ptr + 8+(3*i))+1);
			temp2 = *((ptr + 8+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->DevEUI[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->DevEUI=");
  index=0;
  for(index=0;index<8;index++)
  {
    PNI_PRINTF("%02x",lora_temp->DevEUI[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  // appeui get & wait for resp
  if ((rval = xdot_at_cmd_get_appeui(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appeui get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get appeui Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+APPEUI:")) != NULL)
  {
		for(uint8_t i=0;i<8;i++)
		{
			temp1 = *((ptr + 8+(3*i))+1);
			temp2 = *((ptr + 8+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->AppEUI[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->AppEUI=");
  index=0;
  for(index=0;index<8;index++)
  {
    PNI_PRINTF("%02x",lora_temp->AppEUI[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  // devaddr get & wait for resp
  if ((rval = xdot_at_cmd_get_devaddr(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("devaddr get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get devaddr Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+DEVADDR:")) != NULL)
  {
		for(uint8_t i=0;i<4;i++)
		{
			temp1 = *((ptr + 9+(3*i))+1);
			temp2 = *((ptr + 9+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->DevADDR[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->DevADDR=");
  index=0;
  for(index=0;index<4;index++)
  {
    PNI_PRINTF("%02x",lora_temp->DevADDR[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  // appkey get & wait for resp
  if ((rval = xdot_at_cmd_get_appkey(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appkey  get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get appkey  Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+APPKEY:")) != NULL)
  {
		for(uint8_t i=0;i<16;i++)
		{
			temp1 = *((ptr + 8+(3*i))+1);
			temp2 = *((ptr + 8+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->AppKEY[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->AppKEY=");
  index=0;
  for(index=0;index<16;index++)
  {
    PNI_PRINTF("%02x",lora_temp->AppKEY[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  // appskey get & wait for resp
  if ((rval = xdot_at_cmd_get_appskey(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appskey  get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get appskey  Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+APPSKEY:")) != NULL)
  {
		for(uint8_t i=0;i<16;i++)
		{
			temp1 = *((ptr + 9+(3*i))+1);
			temp2 = *((ptr + 9+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->AppSKEY[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->AppSKEY=");
  index=0;
  for(index=0;index<16;index++)
  {
    PNI_PRINTF("%02x",lora_temp->AppSKEY[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  // nwkskey get & wait for resp
  if ((rval = xdot_at_cmd_get_nwkskey(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("nwkskey  get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get nwkskey  Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+NWKSKEY:")) != NULL)
  {
		for(uint8_t i=0;i<16;i++)
		{
			temp1 = *((ptr + 9+(3*i))+1);
			temp2 = *((ptr + 9+(3*i))+2);
			if(temp1 > 0x40)
			{
				temp1 = temp1 - 55;
			}else{
					temp1 = temp1 - 48;
				}
			if(temp2 > 0x40)
			{
				temp2 = temp2 - 55;
			}else{
					temp2 = temp2 - 48;
				}
			lora_temp->NwkSKEY[i] = temp1*16 + temp2;
		}
	}

  #if XDOT_ENABLE_DEBUG
  PNI_PRINTF("lora_temp->NwkSKEY=");
  index=0;
  for(index=0;index<16;index++)
  {
    PNI_PRINTF("%02x",lora_temp->NwkSKEY[index]);
  }
  PNI_PRINTF("\r\n");
  #endif

  return rval;
}

static int xdot_get_Init_node_info(Node_Info *lora_temp)
{
  char *ptr = NULL;
  int rval = PNI_RADIO_RET_OK;
  Xdot_AtResp resp = { 0 };

  // otaa get & wait for resp
  if ((rval = xdot_at_cmd_get_otaa(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("otaa get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get OTAA Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);


  if((ptr =xdot_string_str((char *)resp.data,"+OTAA:")) != NULL)
  {
    if(*(ptr + 7) == 0x30)
		{
			lora_temp->NET_Mode= 0;
		}
		if(*(ptr + 7) == 0x31)
		{
			lora_temp->NET_Mode= 1;
		}
  }

  // comfirm get & wait for resp
  if ((rval = xdot_at_cmd_get_confirm(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("otaa get");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("get OTAA Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);
  if((ptr =xdot_string_str((char *)resp.data,"+CONFIRM:")) != NULL)
  {
		if(*(ptr + 10) == 0x30)
		{
			lora_temp->Confirm= 0;
		}
		if(*(ptr + 10) == 0x31)
		{
			lora_temp->Confirm= 1;
		}
	}

  XDOT_D("LoRaNodestr->NET_Mode=%d",LoRaNodestr->NET_Mode);
  XDOT_D("LoRaNodestr->Confirm=%d",LoRaNodestr->Confirm);

  return rval;
}

static void xdot_node_sta_check(Node_Info *LoRa_temp)
{
  static uint8_t online_log = 0;
  if(NET_ERROR == xdot_stat_status_check())
  {LoRa_temp->ONline = 0;}
  else
  {LoRa_temp->ONline = 1;}

  if(MODULE_BUSY== xdot_busy_status_check())
  {LoRa_temp->BUSY = 0;}
  else
  {LoRa_temp->BUSY = 1;}

  if(LoRa_temp->NET_Mode == 1)
  {//OTAA
    if(LoRa_temp->ONline == 1)
    {
      if(LoRa_temp->BUSY == 1)
      {LoRa_temp->Join = 1;}
    }
    else
    {LoRa_temp->Join = 0;}
  }
  else if(LoRa_temp->NET_Mode == 0)
  {//ABP
    if(LoRa_temp->BUSY == 1)
    {LoRa_temp->Join = 1;}
    else
    {LoRa_temp->Join = 0;}
  }

  if(LoRa_temp->Join != online_log )
  {
    if(LoRa_temp->Join == 1)
    {
      XDOT_D("network normal !!!");

    }
    else
    {
      XDOT_D("network anomaly !!!");
    }
  }
  online_log = LoRa_temp->Join;
}


static int xdot_Abp_Config(void)
{
  int rval=PNI_RADIO_RET_OK;
  Xdot_AtResp resp = { 0 };
  const char *appskey={" 2B 7E 15 16 28 AE D2 A6 AB F7 15 88 09 CF 4F 3C"};
  const char *nwkskey={" 2B 7E 15 16 28 AE D2 A6 AB F7 15 88 09 CF 4F 3C"};

  // work mode
  if ((rval = xdot_mode_mcu(MODE_CMD)) != PNI_RADIO_RET_OK)
  {
    XDOT_E("mode MCU");
    return rval;
  }

  // otaa config & wait for resp
  if ((rval = xdot_at_cmd_set_otaa(&m_xdot_at_cmd_iface,0)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("otaa set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("OTAA Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // appskey config & wait for resp
  if ((rval = xdot_at_cmd_set_appskey(&m_xdot_at_cmd_iface,appskey)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appeui set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("APPEUI Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // nwkskey config & wait for resp
  if ((rval = xdot_at_cmd_set_nwkskey(&m_xdot_at_cmd_iface,nwkskey)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appkey set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("APPKEY Waiting for response");
      //return rval;//return 'bad param',spilt it.
  }
  XDOT_D("%s",resp.data);

  // save config & wait for resp
  if ((rval = xdot_at_cmd_set_save(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("save set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 5000)) != PNI_RADIO_RET_OK) {
      XDOT_E("SAVE Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // reset MCU: send reset CMD & do the NRST gpio toggle.
  if ((rval = xdot_reset_mcu()) != PNI_RADIO_RET_OK) {
      XDOT_E("Reset MCU");
      return rval;
  }

  return rval;
}

static int xdot_Otaa_Config(Xdot_Cfg *cfg)
{
  int rval=PNI_RADIO_RET_OK;
  Xdot_AtResp resp = { 0 };

  const char *appeui={"2c 26 c5 03 8f 00 00 01"};
  const char *appkey={" 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF"};

  if ((rval = xdot_mode_mcu(MODE_CMD)) != PNI_RADIO_RET_OK)
  {
    XDOT_E("mode MCU");
    return rval;
  }
  // otaa config & wait for resp
  if ((rval = xdot_at_cmd_set_otaa(&m_xdot_at_cmd_iface,1)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("otaa set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("OTAA Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // appeui config & wait for resp
  if ((rval = xdot_at_cmd_set_appeui(&m_xdot_at_cmd_iface,appeui)) != PNI_RADIO_RET_OK)
  {

      XDOT_E("appeui set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("APPEUI Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // appkey config & wait for resp
  if ((rval = xdot_at_cmd_set_appkey(&m_xdot_at_cmd_iface,appkey)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("appkey set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 100)) != PNI_RADIO_RET_OK) {
      XDOT_E("APPKEY Waiting for response");
      //return rval;//return 'bad param',spilt it.
  }
  XDOT_D("%s",resp.data);

  if ((rval = xdot_set_post_join_settings(cfg)) != PNI_RADIO_RET_OK) {
    XDOT_E("DATA RATE Waiting for response");
  }
XDOT_E("szhang add rval=%x\r\n",rval);
  // save config & wait for resp

if((rval =xdot_at_cmd_set_confirm(&m_xdot_at_cmd_iface,1))!= PNI_RADIO_RET_OK)

  {
      XDOT_E("save set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 5000)) != PNI_RADIO_RET_OK) {
      XDOT_E("SAVE Waiting for response");
      return rval;
  }

  if((rval =xdot_at_cmd_set_nbtrials(&m_xdot_at_cmd_iface,8))!= PNI_RADIO_RET_OK)

  {
      XDOT_E("save set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 5000)) != PNI_RADIO_RET_OK) {
      XDOT_E("SAVE Waiting for response");
      return rval;
  }

    if ((rval = xdot_at_cmd_set_save(&m_xdot_at_cmd_iface)) != PNI_RADIO_RET_OK)
  {
      XDOT_E("save set");
      return rval;
  }
  xdot_at_resp_reset(&resp);
  if ((rval = xdot_resp_wait(&resp, 5000)) != PNI_RADIO_RET_OK) {
      XDOT_E("SAVE Waiting for response");
      return rval;
  }
  XDOT_D("%s",resp.data);

  // reset MCU: send reset CMD & do the NRST gpio toggle.
 if ((rval = xdot_reset_mcu()) != PNI_RADIO_RET_OK) {
      XDOT_E("Reset MCU");
      return rval;
  }

  return rval;
}

static int xdot_set_tx_power(void *self, uint8_t dbm)
{
    Xdot_AtResp resp = { 0 };
    int ret;

    if ((ret = xdot_at_cmd_set_tx_power(&m_xdot_at_cmd_iface, dbm))
            != PNI_RADIO_RET_OK) {
        XDOT_E("Set TX power to %u dBm", dbm);
        return ret;
    }

    /* wait for resp */
    xdot_at_resp_reset(&resp);
    if ((ret = xdot_resp_wait(&resp, 20)) != PNI_RADIO_RET_OK) {
        XDOT_E("Waiting for response");
        return ret;
    }

    return PNI_RADIO_RET_OK;
}


static int xdot_setup(Xdot_Cfg *cfg)
{
    int ret;
	XDOT_E("szhang add xdot_setup\r\n");
    // reset MCU: send reset CMD & do the NRST gpio toggle.
    if ((ret = xdot_reset_mcu()) != PNI_RADIO_RET_OK) {
        XDOT_E("Reset MCU");
        return ret;
    }
    // wake mcu
    if ((ret = xdot_wake_mcu(MODULE_RESUME)) != PNI_RADIO_RET_OK)
    {
      XDOT_E("wake MCU");
      return ret;
    }

    //work mode & oTaa config
    if ((ret = xdot_Otaa_Config(cfg)) != PNI_RADIO_RET_OK)
    {
      XDOT_E("otaa MCU");
      return ret;
    }
    xdot_get_Init_node_info(LoRaNodestr);

    // passthrough mode
    if ((ret = xdot_mode_mcu(MODE_PASSTHROUGH)) != PNI_RADIO_RET_OK)
    {
      XDOT_E("mode MCU");
      return ret;
    }
    Clock_Wait(200);
    return PNI_RADIO_RET_OK;
}

static int xdot_configure(void *self, void *params)
{
    PNI_RadioIface *iface = self;
    Xdot_Cfg *cfg = params;

    iface->params = params;

    return xdot_setup(cfg);
}

int pni_radio_xdot_init(PNI_RadioIface *iface, UART_HandleTypeDef *huart)
{
    //Get uart id which ST mcu used.
    m_xdot_huart = huart;
    /*Initialize the Lora operation interface*/
    iface->send = &xdot_send;/*Lora Send data to server*/
    iface->recv = &xdot_recv;/*Lora Recv data to server*/
    iface->sleep = &xdot_sleep;/*Lora Sleep control*/
    iface->reset = &xdot_reset_mcu;/*Lora Reset control*/
    iface->get_device_id = &xdot_get_device_id;/*Get the xDot's unique, 8-byte LoRa node id, Need get it form zte*/
    iface->configure = &xdot_configure;
    iface->set_tx_power = &xdot_set_tx_power;

    /*init xdot_at_cmd_iface(TX inface),it include some flags to judge before send the data.*/
    m_xdot_at_cmd_iface.buffer = m_xdot_buff_tx;//TX buffer
    m_xdot_at_cmd_iface.size = sizeof(m_xdot_buff_tx);// TX len
    m_xdot_at_cmd_iface.exec = &xdot_at_cmd_exec;//Lora TX inface，it include some flags to judge before send the data.


    /*init resp value:1、OK  2、ERROR  3、AT+RECV*/
    xdot_at_resp_hash_init();

    /*init uart1 receive buffer.*/
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(XDOT_NRST_GPIO_Port, XDOT_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    __HAL_UART_FLUSH_DRREGISTER(huart);
    HAL_UART_Receive_DMA(huart, (uint8_t *)&m_xdot_buff_rx,
            sizeof(m_xdot_buff_rx));
    /*setup Lora*/
    return PNI_RADIO_RET_OK;
}
