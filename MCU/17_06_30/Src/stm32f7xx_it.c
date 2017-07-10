/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */
//flags
extern char flag_spi1_rx;
extern char flag_adc_fin;
extern char flag_rock_received;

extern int spi1_temp;
extern int spi1_temp2;
extern int SPI4_CS[3];
int SPI4_data[3];
extern float SPI4_MPPT, SPI4_BATT, SPI4_MOTOR;
extern uint32_t SPI4_sum_data[3];
extern int SPI4_index;
int SPI4_filter_index;
char Uart5_temp_char;
extern int check_cnt;
extern int check_cnt2;
char debug_buffer[1000]={0};
char debug_index=0;

struct que_struct
{
  volatile char data[100];
  volatile char front;
  volatile char rear;
};

extern struct rock_things
{
  volatile int speed_kmph;
  volatile int batt_soc;
  volatile int16_t motor_power;
  volatile char recieve_buffer[10];
  volatile char recieve_index;
} Rock;

extern struct control_data
{
  volatile int throtle;
  volatile float speed;
  volatile float motor_power;
} Control;

// structs for que
extern struct que_struct Lora_que;
extern struct que_struct Gyro_que;
extern struct que_struct Rock_que;

volatile int temp_SR;
volatile char alt=0;
volatile int dac_data=0;
volatile int record_adc[3800]={0};

extern char que_pop(struct que_struct *Q);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim11;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
  int temp_char;
  
  //uart
  if( (Lora_que.front == Lora_que.rear)&&(Gyro_que.front == Gyro_que.rear) ) // all que is empty.
  {
    TIM11->CR1 &= ~(0x1); //interrupt off
    TIM11->CNT = 0;
  }else
  {
    if((Lora_que.front != Lora_que.rear))       //Lora
    {
      temp_char = que_pop(&Lora_que);
      UART4->TDR = temp_char;
      USART3->TDR = temp_char;
    }
    if((Gyro_que.front != Gyro_que.rear))       //Gyro
    {
      temp_char = que_pop(&Gyro_que);
      //USART3_TDR =temp_char;
    }
    if((Rock_que.front != Rock_que.rear))       //Gyro
    {
      temp_char = que_pop(&Rock_que);
    }
  }
  //
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if((TIM2->SR & (0x01)) == 0x01)   //update -> CS LOW   //0
  {      
    GPIOE->ODR &= ~(SPI4_CS[SPI4_index]);       //index 0 1 2
    SPI4->CR1 |= 0x40;
    /* //for check amp
    DAC->DHR12R1 = dac_data;
    DAC->DHR12R2 = dac_data;
    dac_data++;
    if(dac_data>3700)
    {
      dac_data=3700;
    }
    */
  }
  if( (TIM2->SR & (0x02)) == 0x02)   //compare1 -> SPI start     //300
  {
    check_cnt = TIM2->CNT;       //for check time
    SPI4->CR1 &= ~(0x40);
  }
  if( (TIM2->SR & (0x04)) == 0x04)   //compare2 -> SPI off       //3000
  {
    check_cnt2 = TIM2->CNT;
    SPI4->CR1 &= ~(0x40);
  }
  if( (TIM2->SR & (0x08)) == 0x08)   //compare3 -> CS HIGH       //4800
  {
    //GPIOE->ODR |= (SPI4_CS[SPI4_index]);
  }
  TIM2->SR=0;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  
  if((TIM3->SR & (0x01)) == 0x01)         //set
  {
    if((Rock.recieve_buffer[3])&0x10)     //left on PE0
    {
      GPIOE->BSRR = 0x1;
    }
    if((Rock.recieve_buffer[3])&0x08)     //right on PE1
    {
      GPIOE->BSRR = 0x2;
    }
    if((Rock.recieve_buffer[3])&0x08)     //warning on
    {
      GPIOE->BSRR = 0x3;
    }
  }
  
  if((TIM3->SR & (0x02)) == 0x02)       //compare1  half point
  {
    GPIOE->BSRR = 0x30000;       //off all
  }
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  if(((UART5->ISR)&0x20) == 0x20) //recieve
  {
    Uart5_temp_char=UART5->RDR;
    debug_buffer[debug_index]=Uart5_temp_char;
    debug_index++;
    if(Uart5_temp_char == 0xff) //start byte
    {
      Rock.recieve_index=0;
    }
    Rock.recieve_buffer[Rock.recieve_index]=Uart5_temp_char;
    Rock.recieve_index++;
    if(Uart5_temp_char == 0xf8) //stop byte
    {
      flag_rock_received = 1;
    }
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
* @brief This function handles SPI4 global interrupt.
*/
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */
  SPI4->CR1 &= ~(0x40);
  GPIOE->ODR |= (SPI4_CS[SPI4_index]);
  SPI4_data[SPI4_index] = SPI4->DR;
  SPI4_sum_data[SPI4_index] += SPI4_data[SPI4_index];
  SPI4_index++;
  
  if(SPI4_index == 2)
  {
    SPI4_index=0;
    SPI4_filter_index++;
  }
  
  if(SPI4_filter_index == 100)  //filter finish
  {
    flag_adc_fin=1;
    SPI4_filter_index=0;
    SPI4_MPPT = (float)SPI4_sum_data[0]*0.01;
    SPI4_BATT = (float)SPI4_sum_data[1]*0.01;
    SPI4_MOTOR = (float)SPI4_sum_data[2]*0.01;
    SPI4_sum_data[0]=0;
    SPI4_sum_data[1]=0;
    SPI4_sum_data[2]=0;
  }
  //record_adc[dac_data]=spi1_temp2;    //for check amp
  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
