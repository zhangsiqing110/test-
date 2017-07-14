#ifndef PNI_PARKING_MODULE_H
#define PNI_PARKING_MODULE_H

#include <stdint.h>
#include "parking.h"
#include "parking_buffer.h"
#include "parking_sensor.h"
#include "park_temp_sns.h"
#include "radio.h"
#include "SENtralA2_types.h"

extern uint16_t parkingWakeInterval;
extern uint16_t copyParkingWakeInterval;
extern uint16_t gAlarmAInterval;
extern uint16_t totalMagDataLogTime;
extern uint32_t magDataLogStartTime;
extern RTC_TimeTypeDef magDatalogTimeStruct;

typedef struct pni_parking_module {
    PNI_ParkingBufferIface *buffer;//数据包处理接口
    PNI_ParkingSensorIface *sensor;//sensor状态及功能处理接口
    PNI_RadioIface *radio;//Lora状态及功能设置接口
    PNI_ParkingTemperatureSensorIface *tmp_sns;//温度传感器处理接口
    PNI_ParkingCfgState cfg;//配置信息
    struct {
        PNI_ParkingVersion host;//mcu软件版本信息
        PNI_ParkingVersion sensor;//sensor(RM3100rti)软件版本信息
        PNI_ParkingVersion ble;//蓝牙软件版本信息
        PNI_ParkingVersion lora;//Lora软件版本信息
    } version;
} PNI_ParkingModule;


uint32_t Parking_ParseFrameFromLora(PNI_RadioIface *radio,
        PNI_ParkingSensorIface *sensor, PNI_ParkingBufferIface *buffer);
FunctionalState Parking_SendFrame2Lora(PNI_ParkingBufferIface *parking_buffer,
    PNI_RadioIface *radio, int32_t *loraRsp);
void Parking_CheckRcvFrameFromLora(PNI_ParkingBufferIface *parking_buffer);
void Parking_AlarmAHandler(PNI_ParkingSensorIface *sensor,
        PNI_ParkingTemperatureSensorIface *tmp_sns);
int Parking_ParseFifo(SENtralA2Iface *rti, uint8_t *buffer, uint32_t size,
    PNI_ParkingBufferIface *parking_buffer);

#endif /* PNI_PARKING_MODULE_H */
