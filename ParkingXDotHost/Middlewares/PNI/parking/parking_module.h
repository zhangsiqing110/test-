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
    PNI_ParkingBufferIface *buffer;//���ݰ�����ӿ�
    PNI_ParkingSensorIface *sensor;//sensor״̬�����ܴ���ӿ�
    PNI_RadioIface *radio;//Lora״̬���������ýӿ�
    PNI_ParkingTemperatureSensorIface *tmp_sns;//�¶ȴ���������ӿ�
    PNI_ParkingCfgState cfg;//������Ϣ
    struct {
        PNI_ParkingVersion host;//mcu����汾��Ϣ
        PNI_ParkingVersion sensor;//sensor(RM3100rti)����汾��Ϣ
        PNI_ParkingVersion ble;//��������汾��Ϣ
        PNI_ParkingVersion lora;//Lora����汾��Ϣ
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
