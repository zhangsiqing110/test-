#ifndef XDOT_AT_CMD_H
#define XDOT_AT_CMD_H

#if ENABLE_LIERDA_2N717M91
//AT Commands SETTING
#define XDOT_AT_CMD_SET_OTAA "AT+OTAA"
#define XDOT_AT_CMD_SET_APPEUI "AT+APPEUI"
#define XDOT_AT_CMD_SET_APPKEY "AT+APPKEY"
#define XDOT_AT_CMD_SET_SAVE "AT+SAVE"
#define XDOT_AT_CMD_SET_RESET "AT+RESET"
#define XDOT_AT_CMD_SET_BAUD "AT+BAUD"
#define XDOT_AT_CMD_SET_SYSTEM "AT+SYSTEM"
#define XDOT_AT_CMD_SET_APPSKEY "AT+APPSKEY"
#define XDOT_AT_CMD_SET_NWKSKEY "AT+NWKSKEY"
#define XDOT_AT_CMD_TX_DATARATE "AT+DATARATE"
#define XDOT_AT_CMD_SET_CONFIRM "AT+CONFIRM"
#define XDOT_AT_CMD_SET_NBTRIALS "AT+NBTRIALS"


//AT Commands GET
#define XDOT_AT_CMD_GET_VER "AT+VER"
#define XDOT_AT_CMD_GET_DEVEUI "AT+DEVEUI"
#define XDOT_AT_CMD_GET_APPEUI "AT+APPEUI"
#define XDOT_AT_CMD_GET_DEVADDR "AT+DEVADDR"
#define XDOT_AT_CMD_GET_APPKEY "AT+APPKEY"
#define XDOT_AT_CMD_GET_APPSKEY "AT+APPSKEY"
#define XDOT_AT_CMD_GET_NWKSKEY "AT+NWKSKEY"
#define XDOT_AT_CMD_GET_OTAA "AT+OTAA"
#define XDOT_AT_CMD_GET_CONFIRM "AT+CONFIRM"
#define XDOT_AT_CMD_GET_NBTRIALS "AT+NBTRIALS"
#define XDOT_AT_CMD_GET_STATUS "AT+STATUS"
#define XDOT_AT_CMD_GET_JOIN      "AT+JOIN"


typedef struct xdot_at_cmd_iface {
    char *buffer;
    uint16_t size;
    int (*exec)(char *buffer, uint16_t count);
} Xdot_AtCmdIface;

int xdot_at_cmd_set_noargs(Xdot_AtCmdIface *iface, const char *cmd);
int xdot_at_cmd_set_int(Xdot_AtCmdIface *iface, const char *cmd, int value);
int xdot_at_cmd_set_str(Xdot_AtCmdIface *iface, const char *cmd,
        const char *value);
int xdot_at_cmd_set_str(Xdot_AtCmdIface *iface, const char *cmd,
        const char *value);
int xdot_at_cmd_set_int_ne(Xdot_AtCmdIface *iface, const char *cmd, int value);
int xdot_at_cmd_set_int_w_str(Xdot_AtCmdIface *iface, const char *cmd,
        const int id, const char *s);
int xdot_at_cmd_get(Xdot_AtCmdIface *iface, const char *cmd);



/*SETING*/
#define xdot_at_cmd_set_otaa(iface, value) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SET_OTAA, value))

#define xdot_at_cmd_set_appeui(iface, data) ( \
        xdot_at_cmd_set_str(iface, XDOT_AT_CMD_SET_APPEUI, data))

#define xdot_at_cmd_set_appkey(iface, data) ( \
        xdot_at_cmd_set_str(iface, XDOT_AT_CMD_SET_APPKEY, data))

#define xdot_at_cmd_set_save(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_SET_SAVE))

#define xdot_at_cmd_set_reset(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_SET_RESET))

#define xdot_at_cmd_set_baud(iface, value) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SET_BAUD, value))

#define xdot_at_cmd_set_system(iface, value) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SET_SYSTEM, value))

#define xdot_at_cmd_set_appskey(iface, data) ( \
        xdot_at_cmd_set_str(iface, XDOT_AT_CMD_SET_APPSKEY, data))

#define xdot_at_cmd_set_nwkskey(iface, data) ( \
        xdot_at_cmd_set_str(iface, XDOT_AT_CMD_SET_NWKSKEY, data))

#define xdot_at_cmd_set_data_rate(iface, dr) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_TX_DATARATE, dr))
#define xdot_at_cmd_set_confirm(iface,data) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SET_CONFIRM,data))

#define xdot_at_cmd_set_nbtrials(iface,data) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SET_NBTRIALS,data))

#define xdot_at_cmd_set_tx_power(iface, txp) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_TX_PWR, txp))

/*GET*/
#define xdot_at_cmd_get_ver(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_VER))

#define xdot_at_cmd_get_deveui(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_DEVEUI))

#define xdot_at_cmd_get_appeui(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_APPEUI))

#define xdot_at_cmd_get_devaddr(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_DEVADDR))

#define xdot_at_cmd_get_appkey(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_APPKEY))

#define xdot_at_cmd_get_appskey(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_APPSKEY))

#define xdot_at_cmd_get_nwkskey(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_NWKSKEY))

#define xdot_at_cmd_get_otaa(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_OTAA))

#define xdot_at_cmd_get_confirm(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_CONFIRM))

#define xdot_at_cmd_get_nbtrials(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_NBTRIALS))

#define xdot_at_cmd_get_status(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_STATUS))

#define xdot_at_cmd_get_join(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_GET_JOIN))


//#else

/* General AT Commands */

#define XDOT_AT_CMD_ATTENTION "AT"
#define XDOT_AT_CMD_REQUEST_ID "ATI"
#define XDOT_AT_CMD_RESET_CPU "ATZ"
#define XDOT_AT_CMD_ECHO_MODE "ATE"
#define XDOT_AT_CMD_VERBOSE_MODE "ATV"
#define XDOT_AT_CMD_HW_FLOW_CTRL "AT&K"
#define XDOT_AT_CMD_FACTORY_RESET "AT&F"
#define XDOT_AT_CMD_SAVE_CFG "AT&W"
#define XDOT_AT_CMD_WAKE_PIN "AT+WP"
#define XDOT_AT_CMD_SERIAL_SPEED "AT+IPR"
#define XDOT_AT_CMD_DBG_SERIAL_SPEED "AT+DIPR"
#define XDOT_AT_CMD_DBG_LOG_LEVEL "AT+LOG"

/* Network Managment AT Commands */

#define XDOT_AT_CMD_DEVICE_ID "AT+DI"
#define XDOT_AT_CMD_FREQ_BAND "AT+FREQ"
#define XDOT_AT_CMD_FREQ_SUB_BAND "AT+FSB"
#define XDOT_AT_CMD_PUB_NET_MODE "AT+PN"
#define XDOT_AT_CMD_JOIN_BYTE_ORDER "AT+JBO"
#define XDOT_AT_CMD_JOIN_MODE "AT+NJM"
#define XDOT_AT_CMD_JOIN "AT+JOIN"
#define XDOT_AT_CMD_JOIN_RETRIES "AT+JR"
#define XDOT_AT_CMD_JOIN_DELAY "AT+JD"
#define XDOT_AT_CMD_NET_ID "AT+NI"
#define XDOT_AT_CMD_NET_KEY "AT+NK"
#define XDOT_AT_CMD_AES_ENC "AT+ENC"
#define XDOT_AT_CMD_NET_ADDR "AT+NA"
#define XDOT_AT_CMD_NET_SESSION_KEY "AT+NSK"
#define XDOT_AT_CMD_DATA_SESSION_KEY "AT+DSK"
#define XDOT_AT_CMD_UPLINK_COUNTER "AT+ULC"
#define XDOT_AT_CMD_DOWNLINK_COUNTER "AT+DLC"
#define XDOT_AT_CMD_NET_JOIN_STATUS "AT+NJS"
#define XDOT_AT_CMD_PING "AT+PING"
#define XDOT_AT_CMD_ACK_ENABLE "AT+ACK"
#define XDOT_AT_CMD_NET_LINK_CHECK "AT+NLC"
#define XDOT_AT_CMD_LINK_CHECK_COUNT "AT+LCC"
#define XDOT_AT_CMD_LINK_CHECK_THRD "AT+LCT"
#define XDOT_AT_CMD_SAVE_SESSION "AT+SS"
#define XDOT_AT_CMD_RESTORE_SESSION "AT+RS"
#define XDOT_AT_CMD_PRESERVE_SESSION "AT+PS"

/* Sending and Receiving Packets */

#define XDOT_AT_CMD_CHAN_MASK "AT+CHM"
#define XDOT_AT_CMD_TX_CHAN "AT+TXCH"
#define XDOT_AT_CMD_TX_NEXT "AT+TXN"
#define XDOT_AT_CMD_TIME_ON_AIR "AT+TOA"
#define XDOT_AT_CMD_INJECT_MAC "AT+MAC"
#define XDOT_AT_CMD_STATUS "AT&V"
#define XDOT_AT_CMD_DEV_CLASS "AT+DC"
#define XDOT_AT_CMD_APP_PORT "AT+AP"
#define XDOT_AT_CMD_TX_PWR "AT+POWER"
#define XDOT_AT_CMD_TX_INVERT "AT+TXI"
#define XDOT_AT_CMD_RX_INVERT "AT+RXI"
#define XDOT_AT_CMD_RX_DELAY "AT+RXD"
#define XDOT_AT_CMD_FWD_ECC "AT+FEC"
#define XDOT_AT_CMD_CRC "AT+CRC"
#define XDOT_AT_CMD_ADAPT_DR "AT+ADR"
#define XDOT_AT_CMD_TX_DR "AT+TXDR"
#define XDOT_AT_CMD_SESSION_DR "AT+SDR"
#define XDOT_AT_CMD_REPEAT_PKT "AT+REP"
#define XDOT_AT_CMD_SEND "AT+SEND"
#define XDOT_AT_CMD_SEND_BIN "AT+SENDB"
#define XDOT_AT_CMD_RX_ONCE "AT+RECV"
#define XDOT_AT_CMD_RX_OUTPUT "AT+RXO"
#define XDOT_AT_CMD_DATA_PENDING "AT+DP"
#define XDOT_AT_CMD_TX_WAIT "AT+TXW"
#define XDOT_AT_CMD_RESET_STATS "AT&R"
#define XDOT_AT_CMD_STATS "AT&S"
#define XDOT_AT_CMD_RSSI "AT+RSSI"
#define XDOT_AT_CMD_SNR "AT+SNR"
#define XDOT_AT_CMD_SERIAL_DATA_MODE "AT+SD"
#define XDOT_AT_CMD_STARTUP_MODE "AT+SMODE"
#define XDOT_AT_CMD_SERIAL_CLEAR_ON_ERROR "AT+SDCE"

/* Power Management */

#define XDOT_AT_CMD_SLEEP_MODE "AT+SLEEP"
#define XDOT_AT_CMD_WAKE_MODE "AT+WM"
#define XDOT_AT_CMD_WAKE_INTERVAL "AT+WI"
#define XDOT_AT_CMD_WAKE_DELAY "AT+WD"
#define XDOT_AT_CMD_WAKE_TIMEOUT "AT+WTO"
#define XDOT_AT_CMD_ANT_GAIN "AT+ANT"

/* Testing and Compliance */

#define XDOT_AT_CMD_RX_DR "AT+RXDR"
#define XDOT_AT_CMD_RX_FREQ "AT+RXF"
#define XDOT_AT_CMD_RX_CONT "AT+RECVC"
#define XDOT_AT_CMD_SEND_ON_INTERVAL "AT+SENDI"
#define XDOT_AT_CMD_TX_FREQ "AT+TXF"



#define xdot_at_cmd_factory_reset(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_FACTORY_RESET))

#define xdot_at_cmd_set_echo_enabled(iface, enabled) ( \
        xdot_at_cmd_set_int_ne(iface, XDOT_AT_CMD_ECHO_MODE, !!enabled))

#define xdot_at_cmd_set_fsb(iface, fsb) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_FREQ_SUB_BAND, fsb))

#define xdot_at_cmd_set_public_network_enabled(iface, enabled) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_PUB_NET_MODE, !!enabled))

#define xdot_at_cmd_set_network_join_mode(iface, njm) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_JOIN_MODE, njm))

#define xdot_at_cmd_set_network_id(iface, type, id) ( \
        xdot_at_cmd_set_int_w_str(iface, XDOT_AT_CMD_NET_ID, type, id))

#define xdot_at_cmd_set_network_key(iface, type, key) ( \
        xdot_at_cmd_set_int_w_str(iface, XDOT_AT_CMD_NET_KEY, type, key))

#define xdot_at_cmd_set_tx_data_rate(iface, dr) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_TX_DR, dr))

#define xdot_at_cmd_set_acks_enabled(iface, enabled) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_ACK_ENABLE, !!enabled))

#define xdot_at_cmd_save_cfg(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_SAVE_CFG))

#define xdot_at_cmd_join(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_JOIN))

#define xdot_at_cmd_sleep(iface, mode) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_SLEEP_MODE, mode))

#define xdot_at_cmd_sendb(iface, data) ( \
        xdot_at_cmd_set_str(iface, XDOT_AT_CMD_SEND_BIN, data))

#define xdot_at_cmd_set_rx_output(iface, mode) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_RX_OUTPUT, mode))

#define xdot_at_cmd_recv(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_RX_ONCE))

#define xdot_at_cmd_get_net_join_status(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_NET_JOIN_STATUS))

#define xdot_at_cmd_get_tx_next(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_TX_NEXT))

#define xdot_at_cmd_get_device_id(iface) ( \
        xdot_at_cmd_get(iface, XDOT_AT_CMD_DEVICE_ID))

#define xdot_at_cmd_reset(iface) ( \
        xdot_at_cmd_set_noargs(iface, XDOT_AT_CMD_RESET_CPU))

#define xdot_at_cmd_set_wake_mode(iface, mode) ( \
        xdot_at_cmd_set_int(iface, XDOT_AT_CMD_WAKE_MODE, mode))

#endif

#endif /* XDOT_AT_CMD_H */
