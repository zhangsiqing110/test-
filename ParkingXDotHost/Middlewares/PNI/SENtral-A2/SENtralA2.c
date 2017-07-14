/**
* @file			SENtralA2.h
*
* @brief		SENtralA2
*
* @date			12/08/2016
* @copyright    (C) 2016 PNI Corp
*
*               THIS SOFTWARE IS PROVIDED BY PNI SENSOR CORPORATION "AS IS" AND
*               ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
*               TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
*               PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PNI SENSOR
*               CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
*               NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*               HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*               CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
*               OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
*               EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*               DISCLOSURE TO THIRD PARTIES OR REPRODUCTION IN ANY FORM
*               WHATSOEVER, WITHOUT PRIOR WRITTEN CONSENT, IS STRICTLY
*               FORBIDDEN.
*
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "SENtralA2.h"
#include "stm32l0xx_hal.h"
#include "pni_config.h"
#include "wwdg.h"
#include "ble_clock.h"
#include "string.h"
#include "parking_sensor.h"
#if ENABLE_XDOT_RADIO
#include "xdot.h"
#endif /* ENABLE_XDOT_RADIO */

#define ENABLE_DBG_MSG
#ifdef ENABLE_DBG_MSG
#define SENTRAL_A2_DBG  1
#else
#define SENTRAL_A2_DBG  0
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t A2Interrupt = 0;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

#ifdef ENABLE_SWITCH_SOLUTION
#ifdef ENBALE_GAME_ROTATION_VECTOR
#pragma pack(push,1)
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	int16_t accuracy;
} RotationVectorRaw;
#pragma pack(pop)
typedef struct
{
	float x;
	float y;
	float z;
	float w;
	float accuracy;
} RotationVector;

uint8_t  get_rotation_vector(RotationVector *rv, float scale, uint8_t* buffer)
{
	RotationVectorRaw rawData;
	memcpy(&rawData, &buffer[1], sizeof(rawData));
	rv->x = (float)rawData.x * scale;
	rv->y = (float)rawData.y * scale;
	rv->z = (float)rawData.z * scale;
	rv->w = (float)rawData.w * scale;
	rv->accuracy= (float)rawData.accuracy * ((float)M_PI / powf(2.0f, 15.0f)); //TODO: convert to constant

	return 1;
}

#endif

#if (ENABLE_ORIENTATION || ENBALE_TEST_SENTRALA2)

typedef struct
{
	float x;
	float y;
	float z;
	float extra;
} A2SensorData3Axis;
#pragma pack(push,1)
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t status;
} SensorData3AxisRaw;
#pragma pack(pop)

uint8_t  get_3_axis_sensor_data(A2SensorData3Axis *data, float scale, uint8_t* buffer)
{
	SensorData3AxisRaw rawData;
	memcpy(&rawData, &buffer[1], sizeof(rawData));
	data->x = (float)rawData.x * scale;
	data->y = (float)rawData.y * scale;
	data->z = (float)rawData.z * scale;
	data->extra = rawData.status;

	return 1;
}
#endif
#endif

int SENtralA2_i2c_read(uint8_t reg, uint8_t *data, uint16_t size)
{
  if (HAL_I2C_Mem_Read(&SENTRAL_A2_I2C, SENTRAL_A2_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, data, size, SENTRAL_A2_I2C_TIMEOUT)
      != HAL_OK) {
    return SENA2_RET_ERROR;
  }
  return SENA2_RET_OK;
}

int SENtralA2_i2c_read_value(uint8_t reg, uint8_t *value)
{
  return SENtralA2_i2c_read(reg, value, sizeof(*value));
}

int SENtralA2_i2c_write(uint8_t reg, uint8_t *data, uint16_t size)
{
  if (HAL_I2C_Mem_Write(&SENTRAL_A2_I2C, SENTRAL_A2_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, data, size, SENTRAL_A2_I2C_TIMEOUT)
      != HAL_OK) {
    return SENA2_RET_ERROR;
  }
  return SENA2_RET_OK;
}

int SENtralA2_i2c_write_value(uint8_t reg, uint8_t value)
{
  return SENtralA2_i2c_write(reg, &value, sizeof(value));
}

uint8_t SENtralA2_is_ready()
{
  uint16_t a2I2cAddr = SENTRAL_A2_ADDRESS;
  // check if Sentral is Ready for communication
  HAL_StatusTypeDef x = HAL_I2C_IsDeviceReady(&SENTRAL_A2_I2C,(uint16_t)a2I2cAddr,1,1000);

  while(x != HAL_OK)
  {
    if(x == HAL_ERROR)
    {
      SENA2_E("A2 I2C Device(%d) unable to communicate\r\n", a2I2cAddr);
      Error_Handler();
      return PNI_NO;
    }
    if(x == HAL_BUSY)
    {
      //Clock_Wait(200);
      SENA2_E("A2 I2C device(%d) still busy - Try again\r\n", a2I2cAddr);
      // TODO: handle Sentral reset failure
    }
    x=HAL_I2C_IsDeviceReady(&SENTRAL_A2_I2C,(uint16_t)(0x50>>1),1,1000);
  }

  SENA2_I("a2 I2C started...communicated with device(%d)\r\n", a2I2cAddr);
  return PNI_YES;
}

char* A2strBits(void const * const ptr, uint8_t numBytes, char* str)
{
	uint8_t *bytes = (uint8_t *)ptr;
	uint8_t i, j;
	for (i = 0; i < numBytes; i++)
	{
		for (j = 0; j < 8; j++)
		{
			str[i * 8 + (7 - j)] = bytes[(numBytes - 1) - i] & (1 << j) ? '1' : '0';
		}
	}
	str[numBytes * 8] = '\0';
	return str;
}

void displayDeviceIdRegisters()
{
	uint8_t buf[2];
	char str[17];
	PNI_PRINTF("\r\n------------ Displaying ID Registers -----------\r\n");
	SENtralA2_i2c_read(PRODUCT_ID_REG, buf, 2);
	PNI_PRINTF("Product ID:       % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Revision ID:      % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
}

void displayA2StatusRegisters()
{
	uint8_t buf[4];
	char str[17];
	PNI_PRINTF("\r\n------------ Displaying Status Registers -----------\r\n");
	SENtralA2_i2c_read(CHIP_CONTROL_REG, buf, 4);
	PNI_PRINTF("Chip Control[0x34]:      % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Host Status[0x35]:       % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
	PNI_PRINTF("Interrupt Status[0x36]:  % 5u, %s\r\n", buf[2], A2strBits(&buf[2], sizeof(buf[0]), str));
	PNI_PRINTF("Chip Status[0x37]:       % 5u, %s\r\n", buf[3], A2strBits(&buf[3], sizeof(buf[0]), str));
	SENtralA2_i2c_read(ERR_REG, buf, 4);
	PNI_PRINTF("Err Register[0x50]:      % 5u, %s\r\n", buf[0], A2strBits(&buf[0], sizeof(buf[0]), str));
	PNI_PRINTF("Interrupt State[0x51]:   % 5u, %s\r\n", buf[1], A2strBits(&buf[1], sizeof(buf[0]), str));
	PNI_PRINTF("Debug Value[0x52]:       % 5u, %s\r\n", buf[2], A2strBits(&buf[2], sizeof(buf[0]), str));
	PNI_PRINTF("Debug State[0x53]:       % 5u, %s\r\n", buf[3], A2strBits(&buf[3], sizeof(buf[0]), str));
	SENtralA2_i2c_read(BYTES_REMANING_REG, buf, 2);
	uint16_t *v = (uint16_t *)&buf;
	PNI_PRINTF("Bytes Remaining[0x38]:   % 5u, %s\r\n\n", v[0], A2strBits(&v[0], sizeof(v[0]), str));
}

/**
  * @brief GPIO EXTI callback
  * @param None
  * @retval None
  */
void SENtralA2_GPIO_EXTI_Callback()
{
  //PNI_PRINTF("   A2-->DRDY, A2Interrupt = %u\n", A2Interrupt);
  //PNI_PRINTF("A");
  //A2Interrupt = 1;
  A2Interrupt++;

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

/*******************************************************************************
* Function Name  : SentralWrite
* Description    : Writes a byte of data to sentral
* Input          : - Register address to write to
*                  - Data to write.
* Return         : status of write operation
*******************************************************************************/

HAL_StatusTypeDef SentralWrite(const uint8_t RegAddress, const uint8_t data){


  uint8_t SMB_DATA_OUT[3];

  //SMB_DATA_OUT[0] = ;	// Sentral shifted Slave Address
  SMB_DATA_OUT[0] = RegAddress;	// Register number
  SMB_DATA_OUT[1] = data;	// Data to write

  return HAL_I2C_Master_Transmit(&SENTRAL_A2_I2C, 0x50, SMB_DATA_OUT, 2, 1000);

}
/*******************************************************************************
* Function Name  : SentralRead
* Description    : Reads a byte of data from sentral's address
* Input          : - Sentral's register address to read from
* Return         : status of read operation
*******************************************************************************/

HAL_StatusTypeDef SentralRead(uint8_t RegAddress,uint8_t *data)
{
  data[0] = 0;
  HAL_StatusTypeDef ret  = HAL_OK;

  ret|= HAL_I2C_Master_Transmit(&SENTRAL_A2_I2C, 0x50, &RegAddress, 1, 1000);
  ret|= HAL_I2C_Master_Receive(&SENTRAL_A2_I2C, 0x50, data, 1, 1000);

  return ret;
}

uint32_t SENtralA2_param_read(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
	uint8_t i, paramAck, pageSelectValue;
	uint16_t valIndex = 0, retry;

	for (i = 0; i < numParams; i++)
	{
		retry = 0;
		pageSelectValue = page | (paramList[i].size << 4);
		SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);
		SENtralA2_i2c_write_value(PARAM_REQUEST_REG, paramList[i].paramNo);
		do
		{
			//SENtralA2_delay_ms(1); // remove the delay, unless there is problem with param read
			SENtralA2_i2c_read(PARAM_ACK_REG, &paramAck, 1);

      // retry need to be at least 400 for Tethered Board
			if (paramAck == 0x80 || retry > 400)
			{
				SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
				SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
				//SENtralA2_unlock_mutex(paramIoMutex);
				return 0;
			}
			retry++;
		} while (paramAck != paramList[i].paramNo);
		SENtralA2_i2c_read(PARAM_SAVE_REG, &values[valIndex], paramList[i].size);
		valIndex += paramList[i].size;
	}
	SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
	SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);

	return 1;
}

uint32_t SENtralA2_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
	uint8_t i, paramAck, paramNum, pageSelectValue;
	uint16_t valIndex = 0, retry;
	for (i = 0; i < numParams; i++)
	{
		retry = 0;
		pageSelectValue = page | (paramList[i].size << 4);
		SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, pageSelectValue);

		SENtralA2_i2c_write(PARAM_LOAD_REG, &values[valIndex], (uint16_t)paramList[i].size);
    //Clock_Wait(10);   // don't need the delay unless problem writing params

		paramNum = paramList[i].paramNo | 0x80;
		SENtralA2_i2c_write_value(PARAM_REQUEST_REG, paramNum);
    //Clock_Wait(10);   // don't need the delay unless problem writing params
		do
		{
			//SENtralA2_delay_ms(8); //delay not needed unless problem writing params
      SENtralA2_i2c_read_value(PARAM_ACK_REG, &paramAck);

      // retry need to be at least 400 for Tethered Board
			if (paramAck == 0x80 || retry > 400)
			{
        SENA2_E("param write failed - %d\r\n", paramList[i].paramNo);
				SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
				SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
				return 0;
			}
			retry++;
		} while (paramAck != paramNum);

		valIndex += paramList[i].size;
	}

	SENtralA2_i2c_write_value(PARAM_REQUEST_REG, 0);
	SENtralA2_i2c_write_value(PARAM_PAGE_SELECT_REG, 0);
	return 1;
}

int SENtralA2_getBytesRemaining(uint16_t *bytes)
{
  int result = 0;

  result = SENtralA2_i2c_read(BYTES_REMANING_REG, (uint8_t *)bytes,
      sizeof(*bytes));

  if (result != SENA2_RET_OK) {
    SENA2_E("problem reading BYTES REMAINING register, error = %u!!\n", result);
  }

  return result;
}

static void SENtralA2_handle_meta_event(SENtralA2Iface *iface,
    SENtralA2_MetaEvent *meta_event)
{
  SENA2_I("Meta Event: id: 0x%02X, byte1: 0x%02X, byte2: 0x%02X\r\n",
      meta_event->id, meta_event->bytes[0], meta_event->bytes[1]);

  /* no callback iface set */
  if (iface->meta_event_cb == NULL) {
    return;
  }

  /* exec callbacks */
  switch (meta_event->id) {

    case META_EVENT_FLUSH_COMPLETE:
      if (iface->meta_event_cb->on_flush_complete != NULL) {
        iface->meta_event_cb->on_flush_complete(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_SAMPLE_RATE_CHANGED:
      if (iface->meta_event_cb->on_sample_rate_changed != NULL) {
        iface->meta_event_cb->on_sample_rate_changed(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_POWER_MODE_CHANGED:
      if (iface->meta_event_cb->on_power_mode_changed != NULL) {
        iface->meta_event_cb->on_power_mode_changed(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_ERROR:
      if (iface->meta_event_cb->on_error != NULL) {
        iface->meta_event_cb->on_error(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_SENSOR_EVENT:
      if (iface->meta_event_cb->on_sensor_error != NULL) {
        iface->meta_event_cb->on_sensor_error(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_FIFO_OVERFLOW:
      if (iface->meta_event_cb->on_fifo_overflow != NULL) {
        iface->meta_event_cb->on_fifo_overflow(iface->meta_event_cb,
            meta_event->value);
      }
      break;

    case META_EVENT_DYNAMIC_RANGE_CHANGED:
      if (iface->meta_event_cb->on_dynamic_range_changed != NULL) {
        iface->meta_event_cb->on_dynamic_range_changed(iface->meta_event_cb,
            meta_event->bytes[0]);
      }
      break;

    case META_EVENT_FIFO_WATERMARK:
      if (iface->meta_event_cb->on_fifo_watermark != NULL) {
        iface->meta_event_cb->on_fifo_watermark(iface->meta_event_cb,
            meta_event->value);
      }
      break;

    case META_EVENT_SELF_TEST_RESULT:
      if (iface->meta_event_cb->on_self_test_results != NULL) {
        iface->meta_event_cb->on_self_test_results(
            iface->meta_event_cb, meta_event->bytes[0], meta_event->bytes[1]);
      }
      break;

    case META_EVENT_INITIALIZED:
      if (iface->meta_event_cb->on_initialized != NULL) {
        iface->meta_event_cb->on_initialized(iface->meta_event_cb,
            meta_event->value);
      }
      break;
  }
}

static void SENtralA2_handle_stime(SENtralA2Iface *iface, uint8_t sid,
    uint16_t stime)
{
  switch (sid) {

    case SENSOR_TYPE_TIMESTAMP:
      iface->stime_nw = (iface->stime_nw & 0xFFFF0000) | stime;
      break;

    case SENSOR_TYPE_TIMESTAMP_WAKE:
      iface->stime_wk = (iface->stime_nw & 0xFFFF0000) | stime;
      break;

    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
      iface->stime_nw = ((uint32_t)stime << 16);
      break;

    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
      iface->stime_wk = ((uint32_t)stime << 16);
      break;
  }
}

int SENtralA2_parse_fifo(SENtralA2Iface *iface, uint8_t *buffer, uint16_t size,
    SENtralA2_SensorPacket *sns)
{
  SENtralA2_FifoData *data = (SENtralA2_FifoData *)&buffer[1];

  memset(sns, 0, sizeof(*sns));

  /* set sensor id */
  sns->sid = buffer[0];
  SENA2_D("sns->sid = %d\r\n",sns->sid );
  /* do any special handling of FIFO packets */
  switch (sns->sid) {
    case SENSOR_TYPE_TIMESTAMP:
    case SENSOR_TYPE_TIMESTAMP_WAKE:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
      SENtralA2_handle_stime(iface, sns->sid, data->stime);
      break;

    case SENSOR_TYPE_META:
    case SENSOR_TYPE_META_WAKE:
      SENtralA2_handle_meta_event(iface, &data->meta_event);
      break;
#ifdef ENABLE_SWITCH_SOLUTION
#ifdef ENBALE_GAME_ROTATION_VECTOR
    case SENSOR_TYPE_GAME_ROTATION_VECTOR:
      {
        #if 0
        RotationVectorRaw rawData;
        sns->count = (sizeof(rawData)-4);
        memcpy(&sns->data.bytes, &buffer[1], (sizeof(rawData)-4));
        #else
        RotationVector rotationVector;
        float gameRotationScale = 1.0f / powf(2.0f, 14.0f);
        get_rotation_vector(&rotationVector, gameRotationScale, buffer);
        SENA2_D("Game Rotation Vector: %f, %f, %f, %f, %f\r\n", rotationVector.x, rotationVector.y, rotationVector.z, rotationVector.w, rotationVector.accuracy);
        data->GameData[0]=rotationVector.x;
        data->GameData[1]=rotationVector.y;
        data->GameData[2]=rotationVector.z;
        data->GameData[3]=rotationVector.w;
        sns->count = sizeof(data->GameData);
        memcpy(sns->data.bytes, data, sns->count);
        #endif
      }
      break;
#endif
#ifdef ENABLE_ORIENTATION
    case SENSOR_TYPE_ORIENTATION:
      {
        float orientationScale = 360.0f / powf(2.0f, 15.0f);
        A2SensorData3Axis sensorData;
        get_3_axis_sensor_data(&sensorData, orientationScale, buffer);
        SENA2_D("Orientation: %f, %f, %f, %f\r\n", sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
        //return 8;
        data->OrientationData[0]=sensorData.x;
        data->OrientationData[1]=sensorData.y;
        data->OrientationData[2]=sensorData.z;
        sns->count = sizeof(data->OrientationData);
        memcpy(sns->data.bytes, data, sns->count);
      }
      break;
#endif

#endif

  }

  /* set timestamp */
  if ((sns->sid < SENSOR_TYPE_ACCELEROMETER_WAKE)
      || (sns->sid == SENSOR_TYPE_DEBUG)
      || (sns->sid == SENSOR_TYPE_TIMESTAMP)
      || (sns->sid == SENSOR_TYPE_TIMESTAMP_OVERFLOW)
      || (sns->sid == SENSOR_TYPE_META)
      || (sns->sid == SENSOR_TYPE_RAW_GYRO)
      || (sns->sid == SENSOR_TYPE_RAW_MAG)
      || (sns->sid == SENSOR_TYPE_RAW_ACCEL)) {
    sns->timestamp = iface->stime_nw;
  } else {
    sns->timestamp = iface->stime_wk;
  }
#ifdef ENABLE_SWITCH_SOLUTION
#ifdef ENBALE_GAME_ROTATION_VECTOR
  return sns->count;
#endif
#ifdef ENABLE_ORIENTATION
  return sns->count;
#endif
#else
  /* copy data */
  if (iface->event_sizes[sns->sid] > 0) {
    /* set data size, not including sensor id */
    sns->count = iface->event_sizes[sns->sid] - 1;
    memcpy(sns->data.bytes, data, sns->count);
  }
  return iface->event_sizes[sns->sid];

#endif

}

int SENtralA2_read_fifo(uint8_t *buffer)
{
  // Check number of bytes available
  uint8_t bytesAvailable = 0, bytesRead = 0;
  uint16_t bytesToRead = 0;

  SENtralA2_i2c_read(BYTES_REMANING_REG, (uint8_t *)&bytesAvailable,
      sizeof(bytesAvailable));

  while (bytesAvailable > 0) {
    // Break on 50 byte fifo register block
    bytesToRead = ((bytesRead % 50) + I2C_MAX_READ) > 50
        ? 50 - (bytesRead % 50)
        : I2C_MAX_READ;

    // Make sure we don't read more than is available in the fifo
    bytesToRead = MIN(bytesAvailable, bytesToRead);
    if (SENtralA2_i2c_read(bytesRead % 50, &buffer[bytesRead], bytesToRead)
        != SENA2_RET_OK) {
      SENA2_E("error reading FIFO\r\n");
      return SENA2_RET_ERROR;
    }
    bytesAvailable -= bytesToRead;
    bytesRead += bytesToRead;
  }
  return bytesRead;
}

uint32_t SENtralA2_set_sensor_rate(uint8_t sensorId, uint16_t rate)
{
  uint8_t paramPage = PARAM_PAGE_SENSOR_CONF;
  ParamInfo param[] = { sensorId, sizeof(rate) };
  return SENtralA2_param_write((uint8_t *)&rate, paramPage, param, 1);
}

int SENtralA2_self_test(void *self)
{
  uint8_t val = 0;

  //Enable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY;
  SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);

  // Wait for algo standby
  val = 0;
  do {
    HAL_Delay(10);
    SENtralA2_i2c_read_value(HOST_STATUS_REG, &val);
  } while (!(val & HOST_STATUS_ALGORITHM_STANDBY));

  // Enable self test and disable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= (HOST_IFACE_CTRL_FLAG_REQ_SENSOR_SELF_TEST);
  val &= ~(HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY);
  return SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);
}

static int SENtralA2_set_parameter(void *self, uint8_t page, uint8_t num,
    uint8_t *data, uint8_t size)
{
  ParamInfo info[] = { num, size };

  return SENtralA2_param_write(data, page, info, 1);
}

static int SENtralA2_get_parameter(void *self, uint8_t page, uint8_t num,
    uint8_t *data, uint8_t size)
{
  ParamInfo info[] = { num, size };

  return SENtralA2_param_read(data, page, info, 1);
}

static int SENtralA2_set_sns_rate(void *self, uint8_t sid, uint16_t rate)
{
  return SENtralA2_set_sensor_rate(sid, rate);
}

static int SENtralA2_set_meta_event_ctrl(void *self, uint64_t nonwake,
    uint64_t wake)
{
  ParamInfo info[] = {
    { .paramNo = PARAM_META_EVENT_CONTROL, .size = sizeof(nonwake) },
    { .paramNo = PARAM_WAKE_META_EVENT_CONTROL, .size = sizeof(wake) },
  };
  uint64_t data[] = { nonwake, wake };
  uint8_t val = 0;
  int ret = SENA2_RET_OK;

  //Enable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val |= HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY;
  SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);

  // Wait for algo standby
  val = 0;
  do {
    HAL_Delay(10);
    SENtralA2_i2c_read_value(HOST_STATUS_REG, &val);
  } while (!(val & HOST_STATUS_ALGORITHM_STANDBY));

  // set meta event ctrl params
  SENtralA2_param_write((uint8_t *)data, PARAM_PAGE_SYSTEM, info, 2);
  // ret = SENtralA2_set_parameter(self, PARAM_PAGE_SYSTEM, PARAM_META_EVENT_CONTROL, (uint8_t *)&nonwake, sizeof(nonwake));
  if (ret != SENA2_RET_OK) {
    return ret;
  }

  // disable algo standby
  SENtralA2_i2c_read_value(HOST_INTERFACE_CTRL_REG, &val);
  val &= ~(HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY);
  return SENtralA2_i2c_write_value(HOST_INTERFACE_CTRL_REG, val);
}


FwVersion SENtraceVer;
uint32_t SENtralA2_get_fw_version()
{
  uint8_t buf[8],x;
  uint8_t size = 8;
  uint8_t id = 1;
  int rval=0;
  ParamInfo param[] = { id, size };
  rval=SENtralA2_param_read((uint8_t*)buf, PARAM_PAGE_FW_VER, param, 1);
  memcpy(&SENtraceVer, buf, sizeof(buf));

  PNI_PRINTF("+----rval=%d--------------------------------------+\r\n",rval);
  PNI_PRINTF("| PNI SENtrace FW Version: %.2X.%.2X.%.2X.%.8X  |\r\n",
  SENtraceVer.major, SENtraceVer.minor, SENtraceVer.patch, SENtraceVer.build);
  PNI_PRINTF("+---------------------------------------------+\r\n");
}

int SENtralA2_init(SENtralA2Iface *iface)
{
  iface->set_parameter = &SENtralA2_set_parameter;
  iface->get_parameter = &SENtralA2_get_parameter;
  iface->set_sns_rate = &SENtralA2_set_sns_rate;
  iface->self_test = &SENtralA2_self_test;
  iface->set_meta_event_ctrl = &SENtralA2_set_meta_event_ctrl;

  /* init all sensor event sizes to 0 */
  memset(iface->event_sizes, 0, sizeof(iface->event_sizes));

  /* set standard non-sensor type event sizes */
  /* timestamps */
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_OVERFLOW] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_WAKE] = 3;
  iface->event_sizes[SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE] = 3;

  /* meta events */
  iface->event_sizes[SENSOR_TYPE_META] = 4;
  iface->event_sizes[SENSOR_TYPE_META_WAKE] = 4;

  if (SENtralA2_is_ready() != PNI_YES) {
    SENA2_E("Unable to communicate with SENtral-A2\r\n");
    return SENA2_RET_ERROR;
  }

  SENtralA2_i2c_write_value(RESET_REQ_REG, 0x01);
  Clock_Wait(10);
  displayDeviceIdRegisters();
#ifdef ENABLE_SWITCH_SOLUTION

  uint16_t retry = 0;
  typedef struct
  {
  	uint8_t EEPROM_Detected : 1;
  	uint8_t EEPROM_Upload_Done : 1;
  	uint8_t EEPROM_Upload_Error : 1;
  	uint8_t Firmware_Idle : 1;
  	uint8_t No_EEPROM : 1;
  } ChipStatus;
  ChipStatus status;
  do
	{
		SENtralA2_i2c_read(CHIP_STATUS_REG, (uint8_t*)&status, 1);
		Clock_Wait(10);

		if (retry++ > 100)
		{
			PNI_PRINTF("SENtralA2 not idle after reset\r\n");
			Clock_Wait(3000);
			return SENA2_RET_ERROR;
		}
	} while (!status.Firmware_Idle);
#endif

  SENtralA2_i2c_write_value(CHIP_CONTROL_REG, 1);
  Clock_Wait(10);
  displayA2StatusRegisters();
  Clock_Wait(1000);

  SENtralA2_get_fw_version();

  return SENA2_RET_OK;
}

void SENtralA2_flush_fifo()
{
    uint8_t fifo_flush = 0xff;

    SENtralA2_i2c_write_value(0x32, fifo_flush);
}

#if ENBALE_TEST_SENTRALA2
typedef struct GPOBJ
{
  GPIO_TypeDef* PORT;
  uint32_t PIN;
  uint32_t AF_SEL;
}GPIO_OBJ;

typedef struct
{
	float x;
	float y;
	float z;
	float x_bias;
	float y_bias;
	float z_bias;
	float extra;
} SensorData6Axis;
#pragma pack(push,1)
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t x_bias;
	int16_t y_bias;
	int16_t z_bias;
	uint8_t status;
} SensorData3AxisUncalRaw;
#pragma pack(pop)
typedef struct
{
	uint16_t sampleRate;
	uint16_t dynamicRange;
	uint8_t  status;
} PhysicalSensorStatus;

typedef struct
{
	PhysicalSensorStatus accel;
	PhysicalSensorStatus gyro;
	PhysicalSensorStatus mag;
} PhysicalSensorStatusStruct;


#define         SEN_DRDY_PIN                GPIO_PIN_0
#define         SEN_DRDY_PORT               GPIOA
GPIO_OBJ SEN_DRDY_PA0 = { SEN_DRDY_PORT , SEN_DRDY_PIN};

uint32_t timestampNonWake;
uint32_t timestampWake;

uint8_t  SENtralA2_interrupt()
{
	return HAL_GPIO_ReadPin(SEN_DRDY_PA0.PORT,SEN_DRDY_PA0.PIN);
}

uint8_t  get_3_axis_uncal_sensor_data(SensorData6Axis *data, float scale, uint8_t* buffer)
{
	SensorData3AxisUncalRaw rawData;
	memcpy(&rawData, &buffer[1], sizeof(rawData));
	data->x = (float)rawData.x * scale;
	data->y = (float)rawData.y * scale;
	data->z = (float)rawData.z * scale;
	data->x_bias = (float)rawData.x_bias * scale;
	data->y_bias = (float)rawData.y_bias * scale;
	data->z_bias = (float)rawData.z_bias * scale;
	data->extra = rawData.status;

	return 1;
}

void getPhysicalSensorStatus(PhysicalSensorStatusStruct *status)
{
  uint8_t buf[16];
	ParamInfo param[] = { PARAM_PHYSICAL_SENSOR_STATUS, 15 };
	SENtralA2_param_read((uint8_t*)buf, PARAM_PAGE_SYSTEM, param, 1);

  status->accel.sampleRate = (buf[1]<<8) | buf[0];
  status->accel.dynamicRange = (buf[3]<<8) | buf[2];
  status->accel.status= buf[4];

  status->gyro.sampleRate = (buf[6]<<8) | buf[5];
  status->gyro.dynamicRange = (buf[8]<<8) | buf[7];
  status->gyro.status= buf[9];

  status->mag.sampleRate = (buf[11]<<8) | buf[10];
  status->mag.dynamicRange = (buf[13]<<8) | buf[12];
  status->mag.status= buf[14];

}

uint32_t SENtrace_parse_next_fifo_block(uint8_t* buffer, uint32_t size)
{
  uint8_t sensorId = buffer[0];
  uint32_t timestamp;
  uint16_t *timestampPtr;

  PNI_PRINTF("\r\nSENtrace_parse_next_fifo_block: buffer[0] = %d, size = %u\r\n",buffer[0],  size);
  if (sensorId < SENSOR_TYPE_ACCELEROMETER_WAKE ||
      sensorId == SENSOR_TYPE_DEBUG ||
      sensorId == SENSOR_TYPE_TIMESTAMP ||
      sensorId == SENSOR_TYPE_TIMESTAMP_OVERFLOW ||
      sensorId == SENSOR_TYPE_META ||
      sensorId == SENSOR_TYPE_RAW_GYRO ||
      sensorId == SENSOR_TYPE_RAW_MAG ||
      sensorId == SENSOR_TYPE_RAW_ACCEL
      )
  {
    timestamp = timestampNonWake;
    timestampPtr = (uint16_t*)&timestampNonWake;
  }
  else
  {
    timestamp = timestampWake;
    timestampPtr = (uint16_t*)&timestampWake;
  }

  switch (sensorId)
  {
  case 0:
  {
    //PNI_PRINTF("Padding: %d\r\n", size);
    return size;
  }
  case SENSOR_TYPE_ACCELEROMETER:
  case SENSOR_TYPE_ACCELEROMETER_WAKE:
  {


    A2SensorData3Axis sensorData;
    PhysicalSensorStatusStruct physicalSensorStatus;
    getPhysicalSensorStatus(&physicalSensorStatus);
    get_3_axis_sensor_data(&sensorData, (9.81f * physicalSensorStatus.accel.dynamicRange / powf(2.0f, 15.0f)), buffer);
    PNI_PRINTF("a %s: %f, %f, %f, %f\r\n", (9.81f * physicalSensorStatus.accel.dynamicRange / powf(2.0f, 15.0f)), sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
    return 8;
  }

  case SENSOR_TYPE_MAGNETIC_FIELD:
  case SENSOR_TYPE_MAGNETIC_FIELD_WAKE:
  {


    A2SensorData3Axis sensorData;
    PhysicalSensorStatusStruct physicalSensorStatus;
    getPhysicalSensorStatus(&physicalSensorStatus);
    get_3_axis_sensor_data(&sensorData, (physicalSensorStatus.mag.dynamicRange / powf(2.0f, 15.0f)), buffer);
    PNI_PRINTF("a %s: %f, %f, %f, %f\r\n", (physicalSensorStatus.mag.dynamicRange / powf(2.0f, 15.0f)), sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
    return 8;
  }
  case SENSOR_TYPE_GYROSCOPE:
  case SENSOR_TYPE_GYROSCOPE_WAKE:
  {


    A2SensorData3Axis sensorData;
    PhysicalSensorStatusStruct physicalSensorStatus;
    getPhysicalSensorStatus(&physicalSensorStatus);
    get_3_axis_sensor_data(&sensorData, ((3.1415927f / 180.0f) * physicalSensorStatus.gyro.dynamicRange / powf(2.0f, 15.0f)), buffer);
    PNI_PRINTF("a %s: %f, %f, %f, %f\r\n", ((3.1415927f / 180.0f) * physicalSensorStatus.gyro.dynamicRange / powf(2.0f, 15.0f)), sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
    return 8;
  }
  case SENSOR_TYPE_ORIENTATION:
  case SENSOR_TYPE_ORIENTATION_WAKE:
  {
    A2SensorData3Axis sensorData;
    get_3_axis_sensor_data(&sensorData, (360.0f / powf(2.0f, 15.0f)), buffer);
    PNI_PRINTF("SENSOR_TYPE_ORIENTATION %s: %f, %f, %f, %f\r\n", (360.0f / powf(2.0f, 15.0f)), sensorData.x, sensorData.y, sensorData.z, sensorData.extra);
    return 8;
  }
	case SENSOR_TYPE_ROTATION_VECTOR:
	case SENSOR_TYPE_ROTATION_VECTOR_WAKE:
	case SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE:
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE:
	{
		RotationVector rotationVector;
		get_rotation_vector(&rotationVector, (1.0f / powf(2.0f, 14.0f)), buffer);
		PNI_PRINTF("%s: %f, %f, %f, %f, %f\r\n", (1.0f / powf(2.0f, 14.0f)), rotationVector.x, rotationVector.y, rotationVector.z, rotationVector.w, rotationVector.accuracy);
		return 11;
	}
	case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
	case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
	{
		uint16_t* uPacket = (uint16_t*)&buffer[1];
		timestampPtr[1] = uPacket[0];
		timestampPtr[0] = 0;
		timestamp = *(uint32_t*)timestampPtr;
		return 3;
	}

  case SENSOR_TYPE_META:
  case SENSOR_TYPE_META_WAKE:
  {
 		A2SensorData3Axis sensorData;
		sensorData.x = (float)buffer[1];
		sensorData.y = (float)buffer[2];
		sensorData.z = (float)buffer[3];
		sensorData.extra = 0;

		if (sensorData.x == META_EVENT_INITIALIZED)
		{
		}
		if (sensorData.x == META_EVENT_DYNAMIC_RANGE_CHANGED &&
			sensorData.y == SENSOR_TYPE_ACCELEROMETER ||
			sensorData.y == SENSOR_TYPE_ACCELEROMETER_WAKE ||
			sensorData.y == SENSOR_TYPE_MAGNETIC_FIELD ||
			sensorData.y == SENSOR_TYPE_MAGNETIC_FIELD_WAKE ||
			sensorData.y == SENSOR_TYPE_GYROSCOPE ||
			sensorData.y == SENSOR_TYPE_GYROSCOPE_WAKE)
		{
		}
    if (sensorData.x == META_EVENT_CAL_STATUS_CHANGED ||
        sensorData.x == META_EVENT_STILLNESS_CHANGED ||
        sensorData.x == META_EVENT_ACCEL_CAL)
    {
      //autoWarmStartMetaHandler(sensorData.x, sensorData.y);
    }

		return 4;
  }
  default:
    {
      PNI_PRINTF("Other: %d: %d bytes skipped\r\n", buffer[0], size);
      // parsing error? clear out the rest of the buffer and start clean on the
      // next read.
      return size;
    }
  } // end switch

  //return 0; //never reach here
}

uint32_t SENtrace_parse_fifo(uint8_t* buffer, uint32_t size)
{
  uint32_t index = 0;
  uint32_t bytesUsed;
  uint32_t bytesRemaining = size;


  while (bytesRemaining > 0)
  {
    bytesUsed = SENtrace_parse_next_fifo_block(&buffer[index], bytesRemaining);
    index += bytesUsed;
    bytesRemaining -= bytesUsed;

  }
  return 0;
}


#endif

