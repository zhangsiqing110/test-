#ifndef XDOT_H
#define XDOT_H

#include <stdint.h>
#include "radio.h"
#include "usart.h"

#define XDOT_ENABLE_DEBUG (1)

#if XDOT_ENABLE_DEBUG
#define XDOT_D(fmt, ...) PNI_PRINTF("[DEBUG][XDOT][%s] " fmt "\r\n", __func__, ##__VA_ARGS__)
#else /* XDOT_ENABLE_DEBUG */
#define XDOT_D(fmt, ...)
#endif /* XDOT_ENABLE_DEBUG */

#define XDOT_E(fmt, ...) PNI_PRINTF("[ERROR][XDOT][%s] " fmt "\r\n", __func__, ##__VA_ARGS__)
#define XDOT_I(fmt, ...) PNI_PRINTF("[INFO][XDOT] " fmt "\r\n", ##__VA_ARGS__)

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* MAX */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

/*
 * Join retry threshold. Once this threshold is hit the response delay uses a
 * multiplier
 */
#define XDOT_JOIN_RETRY_THRD (10)

/*
 * Join retry multiplier in ms
 */
#define XDOT_JOIN_RETRY_MULT_MS (10000)

/*
 * TX retry threshold. Once this threshold is hit the response delay uses a
 * multiplier
 */
#define XDOT_TX_RETRY_THRD (10)

/*
 * TX retry multiplier in ms
 */
#define XDOT_TX_RETRY_MULT_MS (1000)

/*
 * Minimum response delay for TX commands
 */
#define XDOT_TX_RESP_DELAY_MIN_MS (1000)

/*
 * Typical xDot AT response time is under 10 ms, with the exception of commands
 * that open RX windows like JOIN, SEND and RECV.
 */
#define XDOT_RESP_DELAY_MS (10)

/*
 * Maximum payload size for AT+SEND (char string)
 */
#define XDOT_SEND_MAX_SIZE (242)

/*
 * Maximum payload size for AT+SENDB (hex string)
 */
#define XDOT_SENDB_MAX_SIZE (XDOT_SEND_MAX_SIZE * 2)

typedef enum xdot_fsb {
    XDOT_FSB_HOP = 0,
    XDOT_FSB_902_3__903_7 = 1,
    XDOT_FSB_903_9__905_3 = 2,
    XDOT_FSB_905_5__906_9 = 3,
    XDOT_FSB_907_1__908_5 = 4,
    XDOT_FSB_908_7__910_1 = 5,
    XDOT_FSB_910_3__911_7 = 6,
    XDOT_FSB_911_9__913_3 = 7,
    XDOT_FSB_913_5__914_9 = 8,
    XDOT_FSB_MAX,
} Xdot_FreqSubBand;

typedef enum xdot_network_join_mode {
    XDOT_NJM_MANUAL = 0,
    XDOT_NJM_OTA = 1, // default
    XDOT_NJM_AUTO_OTA = 2,
    XDOT_NJM_P2P = 3,
} Xdot_NetworkJoinMode;

typedef enum xdot_network_param_type {
    XDOT_NI_TYPE_HEX = 0,
    XDOT_NI_TYPE_STRING = 1,
} Xdot_NetworkParamType;

typedef enum xdot_rx_output_type {
    XDOT_RX_OUT_TYPE_HEX = 0, // default
    XDOT_RX_OUT_TYPE_RAW = 1,
} Xdot_RxOutputType;

typedef enum xdot_data_rate {
    XDOT_DATA_RATE_INVALID = PNI_RADIO_RET_EINVAL,
    XDOT_DATA_RATE_DR0 = 0,
    XDOT_DATA_RATE_DR1 = 1,
    XDOT_DATA_RATE_DR2 = 2,
    XDOT_DATA_RATE_DR3 = 3,
    XDOT_DATA_RATE_DR4 = 4,
    XDOT_DATA_RATE_DR5 = 5,
    XDOT_DATA_RATE_DR6 = 6,
    XDOT_DATA_RATE_DR7 = 7,
    XDOT_DATA_RATE_MAX = 8,
} Xdot_DataRate;

typedef enum xdot_spreading_factor {
    XDOT_SF_12_BW_125,
    XDOT_SF_11_BW_125,
    XDOT_SF_10_BW_125,
    XDOT_SF_9_BW_125,
    XDOT_SF_8_BW_125,
    XDOT_SF_8_BW_500,
    XDOT_SF_7_BW_125,
    XDOT_SF_7_BW_250,
    XDOT_SF_FSK,
    XDOT_SF_MAX,
} Xdot_SpreadingFactor;

typedef enum xdot_ism_band {
    XDOT_ISM_BAND_EU868 = 0,
    XDOT_ISM_BAND_US915 = 1,
    XDOT_ISM_BAND_CH470 = 2,
    XDOT_ISM_BAND_MAX = 3,
} Xdot_IsmBand;

typedef enum xdot_sleep_mode {
    XDOT_SLEEP_MODE_STANDBY = 0,
    XDOT_SLEEP_MODE_STOP = 1,
} Xdot_SleepMode;

typedef enum xdot_wake_mode {
    XDOT_WAKE_MODE_INTERVAL = 0,
    XDOT_WAKE_MODE_INTERRUPT = 1,
} Xdot_WakeMode;

typedef enum xdot_network_join_status {
    XDOT_NET_JOIN_STATUS_NOT_JOINED = 0,
    XDOT_NET_JOIN_STATUS_JOINED = 1,
} Xdot_NetworkJoinStatus;

typedef struct xdot_cfg {
    int echo_enabled;
    Xdot_IsmBand ism_band;
    Xdot_FreqSubBand freq_sub_band;
    int public_network_enabled;
    Xdot_NetworkJoinMode network_join_mode;
    const char *network_app_eui;
    Xdot_NetworkParamType ni_type;
    const char *network_app_key;
    Xdot_NetworkParamType nk_type;
    Xdot_SpreadingFactor sf;
    int acks_enabled;
    Xdot_RxOutputType rx_type;
    uint8_t txp;
} Xdot_Cfg;

typedef enum xdot_state_flag {
    XDOT_STATE_FLAG_NONE = 0,
    XDOT_STATE_FLAG_JOINED =    (1 << 0),
    XDOT_STATE_FLAG_RESP_WAIT = (1 << 1),
    XDOT_STATE_FLAG_RESP_RECV = (1 << 2),
    XDOT_STATE_FLAG_JOIN_WAIT = (1 << 3),
    XDOT_STATE_FLAG_SEND_WAIT = (1 << 4),
    XDOT_STATE_FLAG_IN_SLEEP =  (1 << 5),
} Xdot_StateFlag;

typedef struct xdot_state {
    Xdot_StateFlag flags;
    int join_retry_count;
    int tx_retry_count;
    uint16_t tx_packet_count;
} Xdot_State;

typedef enum {
	NET_ABP=0,
	NET_OTAA,
} Lora_NetMode_T;

typedef enum {
	MODE_PASSTHROUGH=0,
	MODE_CMD,
} Lora_Mode_T;

typedef enum {
	MODULE_SUSPEND=0,
	MODULE_RESUME,
} Lora_WAKE_T;

typedef enum {
	MODULE_BUSY=0,
	MODULE_IDLE,
} Lora_BUSY_T;

typedef enum {
	NET_ERROR=0,
	NET_OK,
} Lora_STAT_T;
typedef struct xdot_rssi {
    float last;
    float min;
    float max;
    float avg;
} Xdot_Rssi;

typedef struct xdot_snr {
    float last;
    float min;
    float max;
    float avg;
} Xdot_Snr;
typedef struct xdot_test_params {
    const char *txf;
    uint8_t txp;
    const char *msg;
    uint32_t interval_ms;
} Xdot_TestParams;

typedef struct {
	uint8_t DevEUI[8];
	uint8_t AppEUI[8];
	uint8_t DevADDR[4];
	uint8_t AppKEY[16];
	uint8_t AppSKEY[16];
	uint8_t NwkSKEY[16];

	uint8_t Join;
	Lora_Mode_T MODE;
	uint8_t ONline;
	uint8_t BUSY;
	uint8_t NET_Mode;
	uint8_t Confirm;
} Node_Info;

/*
 * Convert byte array to hex string with supplied delimiter
 *
 * @param bytes pointer to byte array to read from
 * @param size size of byte array
 * @param hex pointer to char array to write into. this array *must* be at
 *            least size of byte array * 3, e.g. 8-byte input needs 24-byte
 *            output.
 * @param delim delimiter to use
 * @param upper 0 = uppercase chars, 1 = lowercase
 */
void xdot_bytes_to_hex(const uint8_t *bytes, uint16_t size, char *hex,
        const char delim, uint8_t upper);

int pni_radio_xdot_init(PNI_RadioIface *iface, UART_HandleTypeDef *huart);

#endif /* XDOT_H */
