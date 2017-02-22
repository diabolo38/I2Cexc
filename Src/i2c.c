/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
/** if set then any extra host write is handled but data is discarded
 * if nto active then any extra wr may screw up master/slave
 * altertive is to use modified hal to nak write when no more data is needed what is the "i2c slave practise "
 *
 */
#define	I2C_SALVE_WR_TRASH_JUNK	1

#define i2c_debug(...) (void)0
// "index out of range no ack no more recv ");

volatile int i2c_new_data;
struct i2c_acces_t i2c_access;
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 32;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

  }
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

struct i2c_stat_t {
	int listen; // cb cnt
	int error; // cb cnt
	int tx; // cb cnt
	int rx; // cb cnt
	int addr; // cb cnt

	int no_tx_data; // internal tx with index limit host get more data
	int no_rx_data; // internal rx with index limit host put more data
};

#if 1
	volatile struct i2c_stat_t i2c_cnt;
#	define I2C_STAT(x) i2c_cnt.x
#else
#	define I2C_STAT(x) (void)0
#endif

volatile int n_cb=0;
int i2c_last_rx;
uint8_t  i2c_cur_index;

#define N_REG	32
int i2c_max_index = N_REG; /// all index > thsi bad
uint8_t i2c_rx_buffer[256];
uint8_t i2c_reg_buffer[N_REG];
uint8_t i2c_dummy_data[4]={0xDE,0xAD,0xBE, 0xEF};

enum i2c_state_e{
	list_addr=0,
	i2c_index,	// after a addr with wr 1st byte to be rx in
	i2c_rx, // after index rx we do rx in up to  max data we could
	i2c_rx_nodata, // after index rx in  rcv => but is bad => data can be read/wr cos bad index just nack
	i2c_tx, // after index rx in data up to last_rx
	i2c_tx_nodata, // host ask for rd but index is out of range (at max) and we can't send anything
};
enum i2c_state_e i2c_state;

static void i2c_cb(void){
	//for brk and check common path to all cb
	n_cb++;

}

void i2c_fatal(){
	while(1){

	};
}

__weak int i2c_do_in_msg(int index, int n_data,  uint8_t *data){
	return 0;
}

void i2c_handle_wr_done(){
	int n_wr;
	n_wr = i2c_last_rx-hi2c1.XferCount;
	i2c_new_data++;
	i2c_access.rd_wr = 0;
	i2c_access.index = i2c_cur_index;
	i2c_access.n_data = n_wr;
	i2c_do_in_msg(i2c_cur_index, n_wr, i2c_rx_buffer);
	i2c_cur_index += n_wr;

}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(listen++);
	i2c_cb();

	switch (i2c_state){
	case i2c_rx:
		// time  to do smthg with all data receive if any
		i2c_handle_wr_done();
	break;

	case i2c_tx:
		// we can update index with final amount of data sent
		break;

	default:
		// whatever get ready again and be ready
		break;
	}
	// whatever we didi get ready again
	i2c_state = list_addr;
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(error++);
	//TODO listen again ?
	i2c_cb();
	// master wr xfer may end by a last "nack" what is normal we have error code "4"
	// we can handle it now but a "listen complete cb"  will occur anyway after it where we can factorize handling
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(tx++);
	i2c_cb();
	// we have send all data cool ;)
	// TODO go have to go back to listen if list cplt is nto call !
	// we can know how many data effecively sent to host by using "count"

}

static void i2c_wr_trashing(){
	i2c_state = i2c_rx_nodata;
#if I2C_SALVE_WR_TRASH_JUNK
	int rc;
	rc =  HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, i2c_dummy_data,	sizeof(i2c_dummy_data), I2C_LAST_FRAME);
	if( rc != HAL_OK) {
		i2c_fatal();
	}
#endif
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	int n_data;
	I2C_STAT(rx++);
	i2c_cb();
	switch( i2c_state){
	case i2c_index:
		// now we have index we can limit amoutn of data we can accept
		n_data = i2c_max_index - i2c_cur_index;
		if( n_data  > 0 ) {
			i2c_last_rx = n_data;
			if( HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2c_rx_buffer,
					i2c_last_rx, I2C_FIRST_FRAME) != HAL_OK) {
				i2c_fatal();
			}
			i2c_state = i2c_rx;
		}
		else{
			// index out of range
			i2c_debug("in rx no more rx possible cos index");
			I2C_STAT(no_rx_data++);
			i2c_state = i2c_rx_nodata;
			// FIXME maybe we shall listen again or is ok to juts wait "listencompletd"
			// if we do not get data master get crazy and slave hal code keep on looping on sme irq
			//  and maybe is not nakign to hots !
			// so we have to do something like rcv data but discard it at end (base on state)

		}

		break;
	case i2c_rx:
		// full payload handle message
		i2c_handle_wr_done();
		// fall back to dummy read now stop string what host send
		i2c_wr_trashing();
		break;
	default:
		// keep rcv blike above
		I2C_STAT(no_rx_data++);
		i2c_wr_trashing();
		break;
	}
}


void roll_back_index(){
	if( i2c_cur_index >= i2c_max_index  )
		i2c_cur_index=0;
}


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	int n_data;
	int rc;
	I2C_STAT(addr++);
	//i2c_cb();

	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		// can read from idnex up to and of buffer not more
//HAL_I2C_Slave_Sequential_Receive_IT( *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
		// rcv up to max at once
		i2c_state = i2c_index;
		rc = HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_cur_index,1, I2C_FIRST_FRAME);
		if ( rc != HAL_OK) {
			i2c_fatal();
		}

	} else {
		// host rd data sedn up to max we can
		roll_back_index();
		n_data = i2c_max_index - i2c_cur_index;
		if ( n_data <= 0 ){
			//that is a bug shall nevr happen at least we have one byte !
			i2c_debug("host rd no data for index");
			i2c_state = i2c_tx_nodata;

			i2c_fatal();
		}
		i2c_state = i2c_tx;
		rc = HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2c_reg_buffer[i2c_cur_index], n_data, I2C_LAST_FRAME);
		if ( rc != HAL_OK) {
			i2c_fatal();
		}
		// if host do not get all we don't care but i2c is ready for up to full burst
		// note tha HAL slaebv nto host is handling nak properly

	}
}

int i1c_start(){
	i2c_state = list_addr;
	i2c_cur_index = 0;
	return HAL_I2C_EnableListen_IT(&hi2c1);
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
