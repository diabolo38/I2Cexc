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
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t slave_addr=0x20;
uint8_t wr_buffer[256]={0x01, 0x2, 0x3, 0x4, 0x5, 6, 7,8,
		9,10,11,12,13,14,15,
		16,17,18,19,20,21,22,23,
		24,25,26,27,28,29,30,31};
uint8_t rd_buffer[256]={0x11, 0x22, 0x33, 0x44, 0x55};
char pr_buffer[256];
int acc_delay=2; /* ms wait in between acces to leave slave tiem to do debug print */

#ifdef MASTER
void wr_test() {
	int status;
	int i;
//shall nack on 2nd byte
	uart_printf("== mostly  bad wr ==\n ");
#if 0
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 31, 1, wr_buffer, 2, 10000);
	uart_printf("mem wr extra 1 bytes rc %d\n", status);
	HAL_Delay(acc_delay);

	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 31, 1, wr_buffer, 3, 10000);
	uart_printf("mem wr extra 2 bytes rc %d\n", status);
	HAL_Delay(acc_delay);
#endif
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 31, 1, wr_buffer, 5, 10000);
	uart_printf("mem wr extra 4 bytes rc %d\n", status);
	HAL_Delay(acc_delay);
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 31, 1, wr_buffer, 5, 10000);
	uart_printf("mem wr extra 4 bytes rc %d\n", status);
	HAL_Delay(acc_delay);
	for (i = 1; i < 32; i++) {
		uart_printf("mem wr  @31 %d bytes + %d bad ", i, i-1);
		status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 31, 1, wr_buffer, i,	10000);
		uart_printf("status=%d\n", status);
		HAL_Delay(acc_delay);
	}


	// this shall be check on analyzer of device how mnay byte got trashed
	// what is expected is that slave nack real soon at best first bad index
	// master must brake as soon as eeing a nack and report a failure (one data ok)
	uart_printf("== mostly good  ==\n ");
	//many goof btu last bad  bad
	for (i = 1; i < 32; i++) {
		uart_printf("mem wr @d %d bytes ( 1 bad) ", 32-i, i);
		status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 32-i, 1, wr_buffer, i,10000);
		uart_printf("status=%d\n", status);
		HAL_Delay(acc_delay);
	}

	for (i = 1; i < 31; i++) {
		int n=(i%7)+1;
		uart_printf("to mem wr @%d %d bytes ", i, n);
		status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, i, 1, wr_buffer, n,
				10000);
		uart_printf("status=%d\n", status);
		HAL_Delay(acc_delay);
	}
	uart_printf("==repeat bad access==\n");
	for (i = 1; i < 6; i++) {
		int n=i%5;
		uart_printf("to mem wr @%d %d bytes ", i, n);
		status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 33, 1, wr_buffer, n,	10000);
		uart_printf("status=%d\n", status);
		HAL_Delay(acc_delay);
		uart_printf("to mem wr @%d %d bytes ", i, n);
		status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, 33, 1, wr_buffer, n,	10000);
		uart_printf("status=%d\n", status);
		HAL_Delay(acc_delay);
	}
}

void rd_test(){
	int rc;
	int idx,i;
	char *p;
	int test;

	test=0xFFFFFFF;

	if( test&1 ){
		uart_printf("==== rd  good addr ===\n");
		for( idx= 0; idx<32-4; idx++){
			int n = idx%7+1;
			rc = HAL_I2C_Mem_Read(&hi2c1, slave_addr, idx, 1, rd_buffer,n, 10000);
				for(i=0, p=pr_buffer; i<n; i++, p+=3)
					sprintf(p,"%02X ",rd_buffer[i] );
				*p=0;
				uart_printf("mem rd @%d %db status %d data=%s\n", idx, n ,rc,pr_buffer);
				HAL_Delay(acc_delay);
		}
	}
	if( test&2){
		uart_printf("==== wr to invalid addr ===\n");
		for (idx = 2; idx < 5; idx++) {
			rc = HAL_I2C_Mem_Read(&hi2c1, slave_addr, 31, 1, rd_buffer, idx, 10000);
			for (i = 0, p = pr_buffer; i < idx; i++, p += 3)
				sprintf(p, "%02X ", rd_buffer[i]);
			*p = 0;
			uart_printf("mem rd with ext %d status %d data=%s\n", idx - 1, rc, pr_buffer);
			HAL_Delay(acc_delay);
		}
	}
	if ( test&4){
		uart_printf("==== rand wr  + rd ===\n");
		for (idx = 32; idx < 64; idx += 3) {
			int n = idx % 4 + 1;
			rc = HAL_I2C_Mem_Write(&hi2c1, slave_addr, idx, 1, wr_buffer, n, 10000);
			uart_printf("wr @%d %d rc %d\n", idx, n, rc);
			HAL_Delay(acc_delay);
			rc = HAL_I2C_Mem_Read(&hi2c1, slave_addr, idx, 1, rd_buffer, n, 10000);
			for (i = 0, p = pr_buffer; i < n; i++, p += 3)
				sprintf(p, "%02X ", rd_buffer[i]);
			*p = 0;
			uart_printf("rd @%d %db rc=%d data=%s\n", idx, n, rc, pr_buffer);
			HAL_Delay(acc_delay);
		}
	}
	//mix bad and good write see if any is missed
	if ( test&8){
		uart_printf("==== rand wr  + rd ===\n");
		for (idx = 32; idx < 64; idx += 3) {
			int n = idx % 4 + 1;
			int idx2=idx%32;
			rc = HAL_I2C_Mem_Write(&hi2c1, slave_addr, idx, 1, wr_buffer, n, 10000);
			uart_printf("wr @%d %d rc %d\n", idx, n, rc);
			HAL_Delay(acc_delay);
			rc = HAL_I2C_Mem_Write(&hi2c1, slave_addr, idx2, 1, wr_buffer, n, 10000);
			uart_printf("wr @%d %d rc %d\n", idx2, n, rc);
			rc = HAL_I2C_Mem_Write(&hi2c1, slave_addr, idx2, 1, wr_buffer, n, 10000);
			uart_printf("wr @%d %d rc %d\n", idx2, n, rc);
			HAL_Delay(acc_delay);

		}
	}

}
#endif
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int i;
#ifdef MASTER
	int do_wr_test=1;
	int do_rd_test=1;
#endif
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#ifdef MASTER
	  if( do_wr_test )
		  wr_test();
	  if( do_rd_test )
		  rd_test();
#else
	  int status = i1c_start();
	  if( status ) {
		  uart_printf("fail to tsart slave %d\n",status);
	  }
	  while( 1){
		  __WFI();
		  // do some  print
		  if( i2c_new_data ){
			  uart_printf("%d i2c %c idx=%d %d data", i2c_new_data,
					  i2c_access.rd_wr ? 'r':'w' , i2c_access.index, i2c_access.n_data );
			  if( i2c_access.rd_wr == 0 ){
				  extern uint8_t i2c_buffer[32];
				  char *p;
				  for( i=0, p=pr_buffer; i < i2c_access.n_data;  i++){
					  sprintf( p,  "%02x ", i2c_buffer[i]);
					  p+=3;
				  }
				  *p=0;
				  uart_printf(" data=%s\n",pr_buffer);
			  }else{
				  uart_printf("\n",pr_buffer);
			  }
			  _CriticalEnter();
			  i2c_new_data=0;
			  _CriticalExit();
		  }
	  }
#endif
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
