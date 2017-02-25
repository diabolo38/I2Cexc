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
/** if set any extra read pass end of valid index is handed as retrun trash
 * if not set it will hang master in case of such index issue
 */
#define	I2C_SALVE_RD_TRASH_JUNK	1

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

#include <string.h>
struct i2c_stat_t {
	int listen; // cb cnt
	int error; // cb cnt
	int tx; // cb cnt
	int rx; // cb cnt
	int addr; // cb cnt

	int n_wr; // internal rx with index limit host may set more data trashed
	int n_rd; // internal tx with index limit host may get more data undifined

	int no_tx_data; // internal tx with index limit host get more data
	int no_rx_data; // internal rx with index limit host put more data

	int addr_rx_rdy;	// count fixed state in addr  with rx
	int addr_tx_rdy;	// count fixed state in addr  with tx
	int addr_done_rdy;	// count fixed state in over (listen/error cb)
};

#if 1
	volatile struct i2c_stat_t i2c_cnt;
#	define I2C_STAT(x) i2c_cnt.x
#else
#	define I2C_STAT(x) (void)0
#endif

volatile int n_cb=0;
int i2c_last_rx;	/* amount of byte on last rx */
int i2c_last_tx;	/* amount of byte on last tx */
uint8_t  i2c_cur_index;

#define N_REG	32
int i2c_max_index = N_REG; /// all index > thsi bad
uint8_t i2c_buffer[N_REG];
uint8_t i2c_rx_buffer[N_REG];
uint8_t i2c_reg_buffer[N_REG]={
		0, 1, 2, 3 , 4 ,5,6,7,
		8,9,10,11,12,13,14,15,
		16,17,18,19,20,21,22,23,
		24,25,26,27,28,29,30,21};
uint8_t i2c_dummy_data[4]={0xDE,0xAD,0xBE, 0xEF};

enum i2c_state_e{
	list_addr     = 0x00,
	i2c_index     = 0x01,// after a addr with wr 1st byte to be rx in
	i2c_rx        = 0x02, // after index rx we do rx in up to  max data we could
	i2c_rx_nodata = 0x04, // after index rx in  rcv => but is bad => data can be read/wr cos bad index just nack
	i2c_tx        = 0x08, // after index rx in data up to last_rx
	i2c_tx_nodata = 0x10, // host ask for rd but index is out of range (at max) and we can't send anything
};
enum i2c_state_e i2c_state;

/** centralize cb "debug" purpose
 *
 * maibnly to easy brake on n_cb++ data watch point and not use any brake point
 * */
static void i2c_cb(void){
	//for brk and check common path to all cb
	n_cb++;

}

/**
 * any fatal condition
 *
 * hopefully it shall not occur :(
 * shall be replace by some set of fail safe recovery i2c  call
 * like forcing state and getting ready for next start
 */
void i2c_fatal(){
	while(1){

	};
}

/* user call back to latch data on "index rd" point */
void i2c_get_data_cb(int i2c_cur_index, void *buffer, int n_data){
	memcpy(buffer, i2c_reg_buffer+i2c_cur_index, n_data);
}

/* user call back to latch data on "index rd" point */
void i2c_put_data_cb(int i2c_cur_index, void *buffer, int n_data){
	memcpy(i2c_reg_buffer+i2c_cur_index, buffer, n_data);
}


void i2c_handle_wr_done(){
	int n_wr;
	I2C_STAT(n_wr++);
	n_wr = i2c_last_rx-hi2c1.XferCount;

	i2c_access.rd_wr = 0;
	i2c_access.index = i2c_cur_index;
	i2c_access.n_data = n_wr;
	i2c_new_data++;
	if( i2c_new_data == 0 ){
		/* copy lacth what receive in safe buffer and we can keep using buffer juts now*/
		i2c_put_data_cb(i2c_cur_index, i2c_buffer,  n_wr);
	}
	else{

	}
}

/* for debg antest purpose make it (void)0 on real usage*/
void i2c_handle_wr_invalid(int index){
	i2c_access.rd_wr = 0;
	i2c_access.index = index;
	i2c_access.n_data = -1;
	i2c_new_data++;
}

/**
 * end of tx read in host poitn odf view
 *
 *
 */
void i2c_handle_rd_done(){
	int n_rd;
	I2C_STAT(n_rd++);
	n_rd = i2c_last_tx-hi2c1.XferCount; //we can't quite say if last byte was sent or not
	i2c_access.rd_wr = 1;
	i2c_access.index = i2c_cur_index;
	i2c_access.n_data = n_rd;
	i2c_new_data++;
}


void i2c_xfer_over(I2C_HandleTypeDef *hi2c){
	switch (i2c_state){
	case i2c_rx:
		// time  to do smthg with all data receive if any
		i2c_handle_wr_done();
	break;

	case i2c_tx:
		// we can update index with final amount of data sent
		i2c_handle_rd_done();
		break;

	case i2c_rx_nodata:
	case i2c_tx_nodata:
		break;
	default:
		// whatever get ready again and be ready
		break;
	}
	// whatever we did get ready again
	i2c_state = list_addr;
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(listen++);
	i2c_cb();
	i2c_xfer_over(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(error++);
	i2c_cb();
	// master wr xfer may end by a last "nack" what is normal we have error code "4"
	// as we may have forced nak it must be clear when stop occur on error

	i2c_xfer_over(hi2c);
}

void i2c_rd_trashing() {
#if	I2C_SALVE_RD_TRASH_JUNK
	int rc;
	rc = HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, i2c_dummy_data, 1, I2C_LAST_FRAME);
	if( rc != HAL_OK) {
		i2c_fatal();
	}
#endif
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	I2C_STAT(tx++);
	i2c_cb();
	// we have send all data cool ;)
	if( i2c_state == i2c_tx )
		i2c_handle_rd_done();
	// still kepn sednign jubk to make HAL code not hangs
	i2c_state = i2c_tx_nodata;
	i2c_rd_trashing();

}

static void i2c_wr_trashing(){
	i2c_state = i2c_rx_nodata;
#if I2C_SALVE_WR_TRASH_JUNK
	int rc;
	rc =  HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, i2c_dummy_data,	1, I2C_LAST_FRAME);
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
		// now we have index we can limit amount of data we can accept
		// TODO roll_back_index(); or not
		// if host send an invalid index we here fully ignore the write ! for read we do accept "roll over"
		// as is on the uart log it look strange in wr/rd test as only rd appear
		n_data = i2c_max_index - i2c_cur_index;
		if( n_data  > 0 ) {
			i2c_last_rx = n_data;
			if( HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2c_buffer, i2c_last_rx, I2C_FIRST_FRAME) != HAL_OK) {
				i2c_fatal();
			}
			i2c_state = i2c_rx;
		}
		else{
			// index out of range
			i2c_debug("in rx no more rx possible cos index");
			I2C_STAT(no_rx_data++);
			i2c_state = i2c_rx_nodata;
			// FIXME maybe we shall listen again or is ok to just wait "listen or error completed" ?
			// if we do not get data master may get crazy and slave hal code keep on looping on same irq
			//  and maybe is not nakign to host ?!
			// so we have to do something like rcv data but discard it at end (base on state)
			i2c_handle_wr_invalid(i2c_cur_index);
			//if we do write trashing then btf patch in hal i2c s not required btu nevr host will see "nak"
			// somehow depending bytes write in trash we can miss some cb and ahev to consier erro as listen end of xfer  due to error (and no complete)

			//overall it work better when sending master garbage
			i2c_wr_trashing();
		}

		break;
	case i2c_rx:
		// full payload handle message
		i2c_handle_wr_done();
		i2c_state = i2c_rx_nodata;
		i2c_wr_trashing();
		break;

	case i2c_rx_nodata:
		// invalid index or data + nack send get ready for next no trashing host shall have stop
		I2C_STAT(no_rx_data++);
		i2c_wr_trashing();
		break;
	default:
		// keep rcv like above
		I2C_STAT(no_rx_data++);
		i2c_wr_trashing();
		break;
	}
}


void roll_back_index(){
	if( i2c_cur_index >= i2c_max_index  )
		i2c_cur_index%=i2c_max_index;
}


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	int n_data;
	int rc;
	I2C_STAT(addr++);
	i2c_cb();

	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		// rcv up to max at once
		i2c_state = i2c_index;
		/* after a host rd/write that did not consume all data hal state is no "LISTEN"
		 * hence receive would fail let patch state as it is valid to change rx to tx
		 */
		if( hi2c->State != HAL_I2C_STATE_LISTEN){
			hi2c->PreviousState = hi2c->State;
			hi2c->State= HAL_I2C_STATE_LISTEN;
			I2C_STAT(addr_rx_rdy++);
		}
		rc = HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2c_cur_index,1, I2C_FIRST_FRAME);
		if ( rc != HAL_OK) {
			i2c_fatal();
		}

	} else {
		// host rd data send up to max we can
		roll_back_index();
		n_data = i2c_max_index - i2c_cur_index;
/* what below can't happen if roll back does it work it was use for debug  but coudl be required if no roll back */
		if ( n_data <= 0 ){
			//that is a bug shall never happen at least we have one byte !
			i2c_debug("host rd no data for index");
			i2c_state = i2c_tx_nodata;
			//TODO  trash ? from now let die
			i2c_fatal();
		}
		i2c_state = i2c_tx;
		i2c_last_tx = n_data;
		// HAl is not wiling to change fm RX to TX after "restart" if not "listen"
		// But how many rw can't be predict  as when we get  index we can also get wr data straigh
		// so we must put prepare a rx  that is not conumed if we get restart
		// simply patching hal state to accept tramsit is valid here
		if( hi2c->State != HAL_I2C_STATE_LISTEN){
			// for debug purpose we could patch alwway
			hi2c->PreviousState = hi2c->State;
			hi2c->State = HAL_I2C_STATE_LISTEN;
			I2C_STAT(addr_tx_rdy++);

		}
		// user fil the dat to read as once copy require reading live from user "reg space"  cause hazard dur to concurent wr with f/W
		i2c_get_data_cb(i2c_cur_index, i2c_buffer, n_data);

		rc = HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, i2c_buffer, n_data, I2C_LAST_FRAME);
		if ( rc != HAL_OK) {
			i2c_fatal();
		}
		// if host do not get all we don't care but i2c is ready for up to full burst
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
