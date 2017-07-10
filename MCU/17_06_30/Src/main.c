/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//

// flags
volatile char flag_spi1_rx = 0;
volatile char flag_adc_fin = 0;
volatile char flag_rock_received = 0;


volatile int spi1_temp = 0;
volatile int spi1_temp2 = 0;
volatile int check_cnt = 0;
volatile int check_cnt2 = 0;

//for SPI4
const int SPI4_CS[3] = {0x800,0x4000,0x8000};        //MPPT PE11, BATT PE14, MOTOR PE15
volatile float SPI4_MPPT=0, SPI4_BATT=0, SPI4_MOTOR=0;
volatile uint32_t SPI4_sum_data[3] = {0};
volatile int SPI4_index=0;
//***********************************************************//

int Lora_timelaps=0;
struct que_struct
{
  volatile char data[100];
  volatile char front;
  volatile char rear;
};
struct rock_things
{
  volatile int speed_kmph;
  volatile int batt_soc;
  volatile int16_t motor_power;
  volatile char recieve_buffer[10];
  volatile char recieve_index;
} Rock;
struct control_data
{
  volatile int throtle;
  volatile float speed;
  volatile float motor_power;
} Control;

struct que_struct Lora_que;
struct que_struct Gyro_que;
struct que_struct Rock_que;

int num[5];
int numt, i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void que_push(struct que_struct *Q, char c);
char que_pop(struct que_struct *Q);
void que_push_str(struct que_struct *Q, char str[]);
void Lora_transmit(void);
void Rock_transmit(void);
void Rock_recieve(void);
void write_num(struct que_struct *Q, float Num);
void write_timelaps(struct que_struct *Q, int Num);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_DAC_Init();
  MX_TIM11_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_UART5_Init();

  /* USER CODE BEGIN 2 */
    
  GPIOE->ODR |= 0x800;   //PE11 HIGH
  SPI4->CR2 |= 0x40;
  
  TIM3->DIER |= 0x01;   //UIE
  TIM3->DIER |= 0x02;   //CC1IE
  
  TIM2->CR1 |= 0x01;
  TIM2->DIER |= 0x0F;
  TIM2->CCER |= 0x111;
  
  TIM11->CR1 |= 0x1;     //enable TIM11
  TIM11->DIER |= 0x1;    //enable TIM11 intterupt

  //USART3->CR1 |= 0x01;   // enable UART
  //USART3->CR1 |= 0x04;   // enable Receive
  //USART3->CR1 |= 0x08;   // enable Transmit
  //USART3->CR1 |= 0x20;   // enable Receive interrupt
  //USART3->CR1 |= 0x40;   // enable Transmit complete interrupt
  //USART3->CR1 |= 0x80;   // enable Transmit data register empty interrupt

  UART4->CR1 |= 0x01;   // enable UART
  UART4->CR1 |= 0x04;   // enable Receive
  UART4->CR1 |= 0x08;   // enable Transmit
  //UART4->CR1 |= 0x20;   // enable Receive interrupt
  //UART4->CR1 |= 0x40;   // enable Transmit complete interrupt
  //UART4->CR1 |= 0x80;   // enable Transmit data register empty interrupt

  UART5->CR1 |= 0x01;   // enable UART
  UART5->CR1 |= 0x04;   // enable Receive
  UART5->CR1 |= 0x08;   // enable Transmit
  UART5->CR1 |= 0x20;   // enable Receive interrupt
  
  
  DAC->CR |= 0x10001;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    DAC->DHR12R1 = 0;
    DAC->DHR12R2 = 0;
    GPIOE->BSRR=0x20;
    HAL_Delay(1000);
    DAC->DHR12R1 = 1000;
    DAC->DHR12R2 = 1000;
    HAL_Delay(1000);
    DAC->DHR12R1 = 2000;
    DAC->DHR12R2 = 2000;
    GPIOE->BSRR=0x200000;
    HAL_Delay(1000);
    DAC->DHR12R1 = 3000;
    DAC->DHR12R2 = 3000;
    HAL_Delay(1000);
    */
    GPIOE->BSRR=0x20;
    HAL_Delay(1000);
    GPIOE->BSRR=0x200000;
    HAL_Delay(1000);
    Lora_transmit();
    Rock_transmit();
    Rock_recieve();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 3000;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 4800;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 665;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 332;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 6000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE11 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void que_push(struct que_struct *Q, char c)
{
  if(((Q->rear+1)%100) == (Q->front))// que full
  {
  }else
  {
    Q->data[Q->rear] = c;
    Q->rear = (Q->rear+1)%100;
  }
}

char que_pop(struct que_struct *Q)
{
  char temp_index;
  if(Q->front == Q->rear) // que empty
  {
  }else
  {
    temp_index=Q->front;
    Q->front = (Q->front+1)%100;
    return Q->data[temp_index];
  }
  return -1;
}
void que_push_str(struct que_struct *Q, char str[])
{
  int index=0;
  for(index=0 ; index<50 ; index++)        //ÃÖ´ë 50°³
  {
    if(str[index]==0)
    {
      return;
    }else
    {
      que_push(Q,str[index]);
    }
  }
}
void Lora_transmit(void)
{
  if((flag_adc_fin != 1)&&(Lora_que.front != Lora_que.rear))
  {
    return ;
  }
  
  flag_adc_fin=0;
  TIM11->CR1 &= ~(0x1); //timer off
  
  que_push_str(&Lora_que,"AT+DATA=\t");
  write_timelaps(&Lora_que,Lora_timelaps);
  Lora_timelaps++;
  
  /*
  SPI4_MPPT = ((SPI4_MPPT-3102)*6.4453125);
  SPI4_BATT = ((SPI4_BATT-3102)*6.4453125);
  SPI4_MOTOR = ((SPI4_MOTOR-3102)*6.4453125);
  */
  
  //write_num(batt_volatge);
  //que_push_str("[V]\t");

  que_push(&Lora_que,'\t');
  write_num(&Lora_que,SPI4_MPPT);       
  que_push_str(&Lora_que,"A\t");
  write_num(&Lora_que,SPI4_BATT);       
  que_push_str(&Lora_que,"A\t");
  write_num(&Lora_que,SPI4_MOTOR);       
  que_push_str(&Lora_que,"A\t");
  
  
  que_push(&Lora_que, 13);
  que_push(&Lora_que, 10);
  
  TIM11->CR1 |= 0x1;    //timer on
  
}
void Rock_transmit(void)
{
  que_push(&Rock_que,0xff);     //Start Byte

  que_push(&Rock_que,(Rock.speed_kmph)&(0x7F));
  que_push(&Rock_que,(Rock.batt_soc)&(0x7F));
  que_push(&Rock_que,(Rock.motor_power >> 11)&(0x7F));
  que_push(&Rock_que,(Rock.motor_power >> 6)&(0x7F));
  que_push(&Rock_que,(Rock.motor_power)&(0x7F));
  
  que_push(&Rock_que,0xf8);     //Stop Byte
}
void Rock_recieve(void)
{
  
  if(flag_rock_received != 1)
  {
    return ;
  }
  flag_rock_received=0;
  
  
  Control.throtle = ( ( (Rock.recieve_buffer[1])&0x1F )<<5 )|((Rock.recieve_buffer[2])&0x1F);
  
  //LED
  if((Rock.recieve_buffer[3])&0x10)     //left on PE0
  {
    TIM3->CR1 |= 0x1;    //timer on
  }
  if((Rock.recieve_buffer[3])&0x08)     //right on PE1
  {
    TIM3->CR1 |= 0x1;    //timer on
  }
  if((Rock.recieve_buffer[3])&0x04)     //warinig on
  {
    TIM3->CR1 |= 0x1;    //timer on
  }
  if((Rock.recieve_buffer[3])&0x1C == 0)
  {
    GPIOE->BSRR = 0x30000;       //turn off both LED
    TIM3->CR1 &= ~(0x1);        //interrupt off
    TIM3->CNT = 1;              //Timer reset
  }
  
  //horn
  if((Rock.recieve_buffer[3])&0x02)     //horn PE3
  {
    GPIOE->BSRR = 0x08;
  }else
  {
    GPIOE->BSRR = 0x80000;
  }
}
void write_timelaps(struct que_struct *Q, int Num)
{
  que_push(Q,'0'+Num%100000/10000);
  que_push(Q,'0'+Num%10000/1000);
  que_push(Q,'0'+Num%1000/100);
  que_push(Q,'0'+Num%100/10);
  que_push(Q,'0'+Num%10);
}
void write_num(struct que_struct *Q, float Num) //second decimal point//(-/+)xxx.xx
{
  int temp_num=0;
  temp_num=(int)(Num*100);
  if(temp_num<0)
  {
    temp_num *= -1;
    que_push(Q,'-');
  }else
  {
    que_push(Q,'+');
  }
  
  que_push(Q,'0'+temp_num%1000000/100000);
  que_push(Q,'0'+temp_num%100000/10000);
  que_push(Q,'0'+temp_num%10000/1000);
  que_push(Q,'0'+temp_num%1000/100);
  que_push(Q,'.');
  que_push(Q,'0'+temp_num%100/10);
  que_push(Q,'0'+temp_num%10);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
