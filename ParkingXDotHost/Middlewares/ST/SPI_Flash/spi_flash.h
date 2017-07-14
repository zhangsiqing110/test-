/**
  ******************************************************************************
  * @file    spi_flash.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   This file contains all the functions prototypes for the spi_flash
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "pni_config.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI Flash supported commands */  
#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0xD8  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

// micro spi flash
#define sFLASH_M25P128_ID         0x202018
#define sFLASH_M25P64_ID          0x202017
#define sFLASH_M25P16_ID          0x202015

// cypress spi flash
#define sFLASH_S25FL116K_ID       0x014015
#define sFLASH_S25FL132K_ID       0x014016
#define sFLASH_S25FL164K_ID       0x014017

/* M25P FLASH SPI Interface pins  */  
#define sFLASH_SPI                           hspi2

#define sFLASH_SPI_SCK_PIN                   MEM_SCK_Pin
#define sFLASH_SPI_SCK_GPIO_PORT             MEM_SCK_GPIO_Port

#define sFLASH_SPI_MISO_PIN                  MEM_MISO_Pin
#define sFLASH_SPI_MISO_GPIO_PORT            MEM_MISO_GPIO_Port

#define sFLASH_SPI_MOSI_PIN                  MEM_MOSI_Pin
#define sFLASH_SPI_MOSI_GPIO_PORT            MEM_MOSI_GPIO_Port

#define sFLASH_CS_PIN                        MEM_CSN_Pin
#define sFLASH_CS_GPIO_PORT                  MEM_CSN_GPIO_Port

#define sFLASH_PWR_PIN                       MEM_PCTRL_Pin
#define sFLASH_PWR_GPIO_PORT                 MEM_PCTRL_GPIO_Port

/* Exported macro ------------------------------------------------------------*/
/* Select sFLASH: Chip Select pin low */
#define sFLASH_CS_LOW()       HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_RESET);
/* Deselect sFLASH: Chip Select pin high */
#define sFLASH_CS_HIGH()      HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_SET);

/* sFLASH power: power pin low */
#define sFLASH_PWR_OFF()       HAL_GPIO_WritePin(sFLASH_PWR_GPIO_PORT, sFLASH_PWR_PIN, GPIO_PIN_RESET);
/* sFlash power on: power pin high */
#define sFLASH_PWR_ON()      HAL_GPIO_WritePin(sFLASH_PWR_GPIO_PORT, sFLASH_PWR_PIN, GPIO_PIN_SET);

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void sFLASH_DeInit(void);
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
