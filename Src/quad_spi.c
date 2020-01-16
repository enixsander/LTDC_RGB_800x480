#include "quad_spi.h"

extern QSPI_HandleTypeDef hqspi;
__IO uint16_t timer_BUSY_flag = 0;

uint8_t QSPI_WriteEnable()
{
  QSPI_CommandTypeDef     s_command;
  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x06;   //WRITE_ENABLE;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  GPIOC->BSRR |= (GPIO_PIN_11 << 16);   //CS On

  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }
  GPIOC->BSRR |= (GPIO_PIN_11); //CS Off

  HAL_Delay(1);

  QspiWaitStatusReg1_BUSY(100);

  return HAL_OK;
}

uint8_t QspiReadData(uint32_t address, uint32_t size, uint8_t* pData)
{
  QSPI_CommandTypeDef s_command;
 
  /* Initialize the read command */
  s_command.InstructionMode  = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction    = 0xBB;  //Fast Read Dual I/O
  s_command.AddressMode    = QSPI_ADDRESS_2_LINES;
  s_command.AddressSize    = QSPI_ADDRESS_24_BITS;
  s_command.Address      = address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.AlternateBytes   = 0;
  s_command.AlternateBytesSize = 0;
  s_command.DataMode      = QSPI_DATA_2_LINES;
  s_command.DummyCycles    = 4;
  s_command.NbData       = size;
  s_command.DdrMode      = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode      = QSPI_SIOO_INST_EVERY_CMD;
  
  //CS
  GPIOC->BSRR |= (GPIO_PIN_11 << 16);
  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }
  //CS
  GPIOC->BSRR |= (GPIO_PIN_11);
 
  return HAL_OK;
}

uint8_t QspiWriteData(uint32_t address, uint32_t size, uint8_t* pData)
{
  QSPI_CommandTypeDef s_command;

  if (QSPI_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;
  }
 
  /* Initialize the write command */
  s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction        = 0x02;  //Page Program
  s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize        = QSPI_ADDRESS_24_BITS;
  s_command.Address            = address;
  s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = QSPI_DATA_1_LINE;
  s_command.DummyCycles        = 0;
  s_command.NbData             = size;
  s_command.DdrMode            = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
 
  //CS
  GPIOC->BSRR |= (GPIO_PIN_11 << 16);
  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }
 
  /* Reception of the data */
  if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }
  GPIOC->BSRR |= (GPIO_PIN_11); 

  HAL_Delay(3); //page program write 3ms max

  return HAL_OK;
}

uint8_t BSP_QSPI_Erase_Sector(uint32_t BlockAddress)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0x20;//ERASE 4KB;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (QSPI_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;
  }

    //CS
  GPIOC->BSRR |= (GPIO_PIN_11 << 16);
  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    GPIOC->BSRR |= (GPIO_PIN_11);
    return HAL_ERROR;
  }  
  GPIOC->BSRR |= (GPIO_PIN_11);

  HAL_Delay(400);  //Sector erase MAX time = 400ms

  return HAL_OK;
}

uint8_t BSP_QSPI_Erase_Block_64KB(uint32_t BlockAddress, uint16_t countBlock)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = 0xD8;//ERASE 64KB;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  //s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  for(uint16_t i = 0; i < countBlock; i++) {
    if (QSPI_WriteEnable() != HAL_OK)
    {
      return HAL_ERROR;
    }

    GPIOC->BSRR |= (GPIO_PIN_11 << 16); //CS

    s_command.Address = BlockAddress + (65536 * i);
    if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      GPIOC->BSRR |= (GPIO_PIN_11);
      return HAL_ERROR;
    }  
    GPIOC->BSRR |= (GPIO_PIN_11);

    //HAL_Delay(200);  //Sector erase MAX time = 2000ms
    QspiWaitStatusReg1_BUSY(2000);
  }

  return HAL_OK;
}

uint8_t QspiWaitStatusReg1_BUSY(uint16_t timeout)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode  = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction    = 0x05;   //status register 1
  s_command.AddressMode    = QSPI_ADDRESS_NONE;
  s_command.AddressSize    = QSPI_ADDRESS_24_BITS;
  s_command.Address      = 0;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.AlternateBytes   = 0;
  s_command.AlternateBytesSize = 0;
  s_command.DataMode      = QSPI_DATA_1_LINE;
  s_command.DummyCycles    = 0;
  s_command.NbData       = 1;
  s_command.DdrMode      = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode      = QSPI_SIOO_INST_EVERY_CMD;
  
  timer_BUSY_flag = timeout;
  uint8_t data = 0x01;

  while(timer_BUSY_flag && (data & 0x01)) { // 0x02 - Write enable latch, 0x01 - BUSY
    GPIOC->BSRR |= (GPIO_PIN_11 << 16);   //CS
    //Configure the command 
    if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      GPIOC->BSRR |= (GPIO_PIN_11);
      return HAL_ERROR;
    }
    if (HAL_QSPI_Receive(&hqspi, &data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      GPIOC->BSRR |= (GPIO_PIN_11);
      return HAL_ERROR;
    }

    GPIOC->BSRR |= (GPIO_PIN_11);   //CS
  }
  if(data & 0x01)
    return HAL_TIMEOUT;
 
  return HAL_OK;
}
