/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "timers.h"   /* Software timer related API prototypes. */

/* USER CODE BEGIN Includes */
#include "SSD1306_I2C.h"
#include <string.h>
#include "nrf24l01.h"

#define LCD_ENABLE
#define RECEIVER
//#define TRANSMITER

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define SIZE_BUF 256
static int  gPtrCur = 0;
static int  gPtrEnd = 0;
static int  fReadyMsg = 0;
static volatile char strRcv[SIZE_BUF];
static volatile char strParse[SIZE_BUF];

static uint8_t fRadioStarted;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS( 500 )

xSemaphoreHandle semBtnReset;

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/*
 * The callback function assigned to the example software timer as described at
 * the top of this file.
 */
static void vExampleTimerCallback( TimerHandle_t xTimer );

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Esp8266SendCmd(const char* str_cmd);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void _delay_us(int delay)
{
	volatile int cnt = delay*1000;
	while(cnt--);
}

// Выбирает активное состояние (высокий уровень) на линии CE
inline void radio_assert_ce() {
	HAL_GPIO_WritePin(GPIOA, NRF24_CE_Pin, GPIO_PIN_SET); // Установка высокого уровня на линии CE
}

// Выбирает неактивное состояние (низкий уровень) на линии CE
inline void radio_deassert_ce() {
	HAL_GPIO_WritePin(GPIOA, NRF24_CE_Pin, GPIO_PIN_RESET); // Установка низкого уровня на линии CE
}

// Поскольку функции для работы с csn не предполагается использовать в иных файлах, их можно объявить static

// Выбирает активное состояние (низкий уровень) на линии CSN
inline static void csn_assert() {
	HAL_GPIO_WritePin(GPIOA, NRF24_CSN_Pin, GPIO_PIN_RESET); // Установка низкого уровня на линии CSN
}

// Выбирает неактивное состояние (высокий уровень) на линии CSN
inline static void csn_deassert() {
	HAL_GPIO_WritePin(GPIOA, NRF24_CSN_Pin, GPIO_PIN_SET); // Установка высокого уровня на линии CSN
}

// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
uint8_t radio_write_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
  csn_assert();

  uint8_t status;
  //uint8_t _reg = (cmd & 31) | W_REGISTER;
  uint8_t _reg = cmd;
  HAL_SPI_TransmitReceive(&hspi1, &_reg, &status, 1, 100);

  HAL_SPI_Transmit(&hspi1, buf, count, 100);

  csn_deassert();
  return status;
}

// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
uint8_t radio_readreg(uint8_t reg) {
  csn_assert();
  uint8_t _reg = (reg & 31) | R_REGISTER;
  HAL_SPI_Transmit(&hspi1, &_reg, 1, 100);


  _reg = 0xFF;
  uint8_t answ;
  HAL_SPI_TransmitReceive(&hspi1, &_reg, &answ, 1, 100);

  csn_deassert();
  return answ;
}

// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
uint8_t radio_writereg(uint8_t reg, uint8_t val) {
  csn_assert();
  uint8_t status;
  uint8_t _reg = (reg & 31) | W_REGISTER;
  HAL_SPI_TransmitReceive(&hspi1, &_reg, &status, 1, 100);
  HAL_SPI_Transmit(&hspi1, &val, 1, 100);
  csn_deassert();
  return status;
}

// Возвращает размер данных в начале FIFO очереди приёмника
uint8_t radio_read_rx_payload_width() {
  csn_assert();
  uint8_t reg = R_RX_PL_WID;
  HAL_SPI_Transmit(&hspi1, &reg, 1, 100);

  uint8_t answ;
  reg = NOP;
  HAL_SPI_TransmitReceive(&hspi1, &reg, &answ, 1, 100);
  csn_deassert();
  return answ;
}

// Выполняет команду cmd, и читает count байт ответа, помещая их в буфер buf, возвращает регистр статуса
uint8_t radio_read_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
  csn_assert();
  uint8_t status;
  HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
  while (count--) {
	  uint8_t byte;
	  cmd = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &byte, 1, 100);
    *(buf++) = byte;
  }
  csn_deassert();
  return status;
}

// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
uint8_t radio_writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
  return radio_write_buf((reg & 31) | W_REGISTER, buf, count);
}

// Выполняет команду. Возвращает регистр статуса
uint8_t radio_cmd(uint8_t cmd) {
  csn_assert();
  uint8_t status;
  HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, 100);
  csn_deassert();
  return status;
}

uint8_t radio_is_interrupt() {
// использовать этот вариант только в крайних случаях!!!
  return (radio_cmd(NOP) & ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))) ? 1 : 0;
}

// Функция производит первоначальную настройку устройства. Возвращает 1, в случае успеха, 0 в случае ошибки
uint8_t radio_start() {
  // transmit
#ifdef TRANSMITER
  uint8_t self_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Собственный адрес
  uint8_t remote_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}; // Адрес удалённой стороны
#endif

  // receiver
#ifdef RECEIVER
  uint8_t remote_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Собственный адрес
  uint8_t self_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}; // Адрес удалённой стороны
#endif

  uint8_t chan = 3; // Номер радио-канала (в диапазоне 0 - 125)

  radio_deassert_ce();
  for(uint8_t cnt = 100;;) {
    radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)); // Выключение питания
    if (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)))
      break;
    // Если прочитано не то что записано, то значит либо радио-чип ещё инициализируется, либо не работает.
    if (!cnt--)
      return 0; // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
    //_delay_ms(1);
    _delay_us(100);
  }

  radio_writereg(EN_AA, (1 << ENAA_P1)); // включение автоподтверждения только по каналу 1
  radio_writereg(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); // включение каналов 0 и 1
  radio_writereg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); // выбор длины адреса 5 байт
  radio_writereg(SETUP_RETR, SETUP_RETR_DELAY_250MKS | SETUP_RETR_UP_TO_2_RETRANSMIT);
  radio_writereg(RF_CH, chan); // Выбор частотного канала
  radio_writereg(RF_SETUP, RF_SETUP_1MBPS | RF_SETUP_0DBM); // выбор скорости 1 Мбит/с и мощности -18dBm

  radio_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5); // Подтверждения приходят на канал 0
  radio_writereg_buf(TX_ADDR, &remote_addr[0], 5);

  radio_writereg_buf(RX_ADDR_P1, &self_addr[0], 5);

  radio_writereg(RX_PW_P0, 0);
  radio_writereg(RX_PW_P1, 32);
  radio_writereg(DYNPD, (1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
  radio_writereg(FEATURE, 0x04); // разрешение произвольной длины пакета данных

  radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX)); // Включение питания

  volatile uint8_t res = radio_readreg(RF_CH);
  res = radio_readreg(RF_CH);
  res = radio_readreg(RX_PW_P1);
  res = radio_readreg(FEATURE);
  res = radio_readreg(CONFIG);

  return (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX))) ? 1 : 0;
}

// Вызывается, когда превышено число попыток отправки, а подтверждение так и не было получено.
void on_send_error() {
 // TODO здесь можно описать обработчик неудачной отправки
	ssd1306_Draw6x8Str(0, 2, "on_send_error", 0, 0);
}

// Вызывается при получении нового пакета по каналу 1 от удалённой стороны.
// buf - буфер с данными, size - длина данных (от 1 до 32)
void on_packet(uint8_t * buf, uint8_t size) {
	static int fCnt = 0;
	static int fPacketCnt = 0;
	char p[17];

 // TODO здесь нужно написать обработчик принятого пакета
	ssd1306_Draw6x8Str(0, 3, "on_packet", 0, 0);

	fPacketCnt++;
	itoa (fPacketCnt, p, 10);
	ssd1306_Draw6x8Str(0, 4, p, 0, 0);

#ifdef RECEIVER
	if (fCnt) {
		fCnt = 0;
		HAL_GPIO_WritePin(GPIOB, LED_BOARD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, B12_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOB, LED_BOARD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, B12_Pin, GPIO_PIN_RESET);
		fCnt = 1;
	}
#endif

 // Если предполагается немедленная отправка ответа, то необходимо обеспечить задержку ,
 // во время которой чип отправит подтверждение о приёме
 // чтобы с момента приёма пакета до перевода в режим PTX прошло:
 // 130мкс + ((длина_адреса + длина_CRC + длина_данных_подтверждения) * 8 + 17) / скорость_обмена
 // При типичных условиях и частоте МК 8 мГц достаточно дополнительной задержки 100мкс
}

// Помещает пакет в очередь отправки.
// buf - буфер с данными, size - длина данных (от 1 до 32)
uint8_t send_data(uint8_t * buf, uint8_t size) {
  radio_deassert_ce(); // Если в режиме приёма, то выключаем его
  uint8_t conf = radio_readreg(CONFIG);
  if (!(conf & (1 << PWR_UP))) // Если питание по какой-то причине отключено, возвращаемся с ошибкой
  {
	  //volatile uint8_t res = radio_readreg(RF_CH);
	  //res = radio_readreg(CONFIG);
    return 0;
  }
  uint8_t status = radio_writereg(CONFIG, conf & ~(1 << PRIM_RX)); // Сбрасываем бит PRIM_RX

  if (status & (1 << TX_FULL_STATUS))  // Если очередь передатчика заполнена, возвращаемся с ошибкой
    return 0;
  radio_write_buf(W_TX_PAYLOAD, buf, size); // Запись данных на отправку

  radio_assert_ce(); // Импульс на линии CE приведёт к началу передачи
  _delay_us(15); // Нужно минимум 10мкс, возьмём с запасом
  radio_deassert_ce();
  return 1;
}

void check_radio() {
  if (!radio_is_interrupt()) // Если прерывания нет, то не задерживаемся
    return;
  uint8_t status = radio_cmd(NOP);
  radio_writereg(STATUS, status); // Просто запишем регистр обратно, тем самым сбросив биты прерываний

  if (status & ((1 << TX_DS) | (1 << MAX_RT))) { // Завершена передача успехом, или нет,
    if (status & (1 << MAX_RT)) { // Если достигнуто максимальное число попыток
      radio_cmd(FLUSH_TX); // Удалим последний пакет из очереди
      on_send_error(); // Вызовем обработчик
    }
    if (!(radio_readreg(FIFO_STATUS) & (1 << TX_EMPTY))) { // Если в очереди передатчика есть что передавать
      radio_assert_ce(); // Импульс на линии CE приведёт к началу передачи
      _delay_us(15); // Нужно минимум 10мкс, возьмём с запасом
      radio_deassert_ce();
    } else {
      uint8_t conf = radio_readreg(CONFIG);
      radio_writereg(CONFIG, conf | (1 << PRIM_RX)); // Устанавливаем бит PRIM_RX: приём
      radio_assert_ce(); // Высокий уровень на линии CE переводит радио-чип в режим приёма
    }
  }
  uint8_t protect = 4; // В очереди FIFO не должно быть более 3 пакетов. Если больше, значит что-то не так
  while (((status & (7 << RX_P_NO)) != (7 << RX_P_NO)) && protect--) { // Пока в очереди есть принятый пакет
    uint8_t l = radio_read_rx_payload_width(); // Узнаём длину пакета
    if (l > 32) { // Ошибка. Такой пакет нужно сбросить
      radio_cmd(FLUSH_RX);
    } else {
      uint8_t buf[32]; // буфер для принятого пакета
      radio_read_buf(R_RX_PAYLOAD, &buf[0], l); // начитывается пакет
      if ((status & (7 << RX_P_NO)) == (1 << RX_P_NO)) { // если datapipe 1
        on_packet(&buf[0], l); // вызываем обработчик полученного пакета
      }
    }
    status = radio_cmd(NOP);
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  TimerHandle_t xExampleSoftwareTimer = NULL;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  csn_deassert();
  radio_deassert_ce();

  fRadioStarted = radio_start();

  _delay_us(1000);
  _delay_us(1000);

  radio_assert_ce();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  semBtnReset = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* Create the software timer as described in the comments at the top of
      this file. */
      xExampleSoftwareTimer = xTimerCreate(     /* A text name, purely to help
                                              debugging. */
                                              ( const signed char * ) "LEDTimer",
                                              /* The timer period, in this case
                                              1000ms (1s). */
                                              mainSOFTWARE_TIMER_PERIOD_MS,
                                              /* This is a periodic timer, so
                                              xAutoReload is set to pdTRUE. */
                                              pdTRUE,
                                              /* The ID is not used, so can be set
                                              to anything. */
                                              ( void * ) 0,
                                              /* The callback function that switches
                                              the LED off. */
                                              vExampleTimerCallback
                                          );

      /* Start the created timer.  A block time of zero is used as the timer
      command queue cannot possibly be full here (this is the first timer to
      be created, and it is not yet running). */
      xTimerStart( xExampleSoftwareTimer, 0 );
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  //HAL_UART_Transmit(&huart1, (uint8_t*)"Start programm!\r\n", 17, 100);

  /* Enable the UART Parity Error and Data Register not empty Interrupts */
  //__HAL_UART_ENABLE_IT(&huart1, USART_CR1_RXNEIE);

  //CLEAR_BIT(USART1->ISR, USART_ISR_RXNE);
  SET_BIT(USART1->CR1, /*USART_CR1_PEIE |*/ USART_CR1_RXNEIE);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, /*LCD_SPI_RST_Pin|*/NRF24_CSN_Pin|NRF24_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B12_Pin | RELAY_Pin|LED_BOARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_SPI_RS_Pin NRF24_CSN_Pin NRF24_CE_Pin */
  GPIO_InitStruct.Pin = LCD_SPI_RST_Pin|NRF24_CSN_Pin|NRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin LED_BOARD_Pin */
  GPIO_InitStruct.Pin = B12_Pin | RELAY_Pin|LED_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Key_Pin */
  GPIO_InitStruct.Pin = Key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void Esp8266SendCmd(const char* str_cmd)
{
	const char* strCRLF = "\r\n";
	uint8_t str[100];
	int len = strlen(str_cmd);

	strcpy(str, str_cmd);
	strcat(str, strCRLF); len+=2;

	HAL_UART_Transmit(&huart1, (uint8_t*)str, len, 100);
}
/* USER CODE END 4 */

static void vExampleTimerCallback( TimerHandle_t xTimer )
{
	static int fState = 0;
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
#ifdef TRANSMITER
	if (fState) {
		fState = 0;
		HAL_GPIO_WritePin(GPIOB, LED_BOARD_Pin, GPIO_PIN_RESET);
	}
	else {
		fState = 1;
		HAL_GPIO_WritePin(GPIOB, LED_BOARD_Pin, GPIO_PIN_SET);
	}
#endif
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1500);

    //vTaskSuspend(xTaskSensor_Handle);   // остановка этой задачи

    //xTaskResumeFromISR(xTaskSensor_Handle);   // возобновление задачи Sensor
       //portEND_SWITCHING_ISR(pdTRUE);         //  и переключение на нее

    //ssd1306_DrawPixel(i++, 20, 0);
    //Esp8266SendCmd("AT+GMR");
    //osDelay(800);
    //Esp8266SendCmd("AT+CIPMUX?");
  }
  /* USER CODE END 5 */ 
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
	const char cr_char = '\r';
	const char lf_char = '\n';
	static char f_pre_char = 0;

  int i = 0;
  int pos;
  static int fPing = 0;
  /* USER CODE BEGIN StartTask02 */
  osDelay(200);
#ifdef LCD_ENABLE
  ssd1306_Init(2);
  ssd1306_Clear();

  ssd1306_Draw6x8Str(0, 0, "STM32 lcd", 0, 0);

  if (fRadioStarted) {
	  ssd1306_Draw6x8Str(0, 1, "Radio OK", 0, 0);
  }
  else  {
	  ssd1306_Draw6x8Str(0, 1, "Radio FAIL", 0, 0);
  }
#endif

  uint8_t mas[] = {0x55, 0x33, 0x89};

  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    check_radio();
#ifdef LCD_ENABLE
    switch (fPing)
    {
    case 0: ssd1306_Draw6x8Str(64, 1, "\\", 0, 0); break;
    case 1: ssd1306_Draw6x8Str(64, 1, "|", 0, 0);  break;
    case 2: ssd1306_Draw6x8Str(64, 1, "/", 0, 0);  break;
    }
    fPing++;
    fPing %= 3;
#endif


#if 0
    if (fReadyMsg) {
    	pos = 0;

    	while (1) {
    	 if (gPtrCur != gPtrEnd) {
    		char sym = strRcv[gPtrCur];
         	gPtrCur+=1;
         	gPtrCur &= 0xFF;

			if ( (sym == lf_char) && (f_pre_char == cr_char)) {
				strParse[pos-1] = '\0';
				break;
			}
			else {
				f_pre_char = sym;
				strParse[pos] = sym;
				pos+=1;
				pos &= 0xFF;
			}
    	 }
    	 else {
    		 pos = 0;
    		 break;
    	 }
    	}
    	if (pos != 0) {
    	    ssd1306_Draw6x8Str(0, 2, (const char*)strParse, 0, 0);
    	}
    }
#endif
  }
  /* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint8_t mas[] = {0x55, 0x33, 0x89};
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(semBtnReset, portMAX_DELAY);

	uint8_t status;
	status = send_data(mas, sizeof(mas));

	osDelay(250);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


#ifdef LCD_ENABLE
    	if (status) {
    		ssd1306_Draw6x8Str(0, 4, "send_data OK", 0, 0);
    	}
    	else {
    		ssd1306_Draw6x8Str(0, 4, "send_data FAIL", 0, 0);
    	}
#endif

  }
  /* USER CODE END StartTask03 */
}

/*
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static BaseType_t xHigherPriorityTaskWoken;

	if (GPIO_Pin == GPIO_PIN_8) {

		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		/* Is it time for vATask() to run? */
		xHigherPriorityTaskWoken = pdFALSE;

		/* Unblock the task by releasing the semaphore. */
		xSemaphoreGiveFromISR( semBtnReset, &xHigherPriorityTaskWoken );

	    /* If xHigherPriorityTaskWoken was set to true you
	    we should yield.  The actual macro used here is
	    port specific. */
	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}


/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
	const char cr_char = '\r';
	const char lf_char = '\n';
	static char f_pre_char = 0;

	volatile int t = 0;
	/* USER CODE BEGIN USART1_IRQn 0 */
	//static portBASE_TYPE xHigherPriorityTaskWoken;
	//xHigherPriorityTaskWoken = pdFALSE;

	char rxc; // received character
	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	rxc = (char)USART1->DR; // Read character from UART

	if ( (rxc == lf_char) && (f_pre_char == cr_char)) {
			t++;
			fReadyMsg = 1;
	}
	f_pre_char = rxc;

	strRcv[gPtrEnd] = rxc;
	gPtrEnd += 1;
	if (gPtrEnd > SIZE_BUF) {
		gPtrEnd = 0;
	}
	//xQueueSendFromISR(xQueueSerIn, &rxc, 0);
	//xQueueSendToFrontFromISR(xQueueSerIn, &rxc, &xHigherPriorityTaskWoken);
	//if( xHigherPriorityTaskWoken == pdTRUE )
	//{
	//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken)
	//	t = 4;
	//}
	/* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
