/*
 * systick.h
 *
 *  Created on: May 10, 2022
 *      Author: dlhyl
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_


#define SYSTICK_BASEADDR   0xE000E010


typedef struct
{
	volatile uint32_t CSR;  //SysTick Control and Status Register
	volatile uint32_t RVR;  //SysTick Reload Value Register
	volatile uint32_t CVR; //SysTick Current Value Register
	volatile uint32_t CALIB;    //SysTick Calibration Value Register

}SysTick_RegDef_t;


SysTick_RegDef_t *pSysTick= ((SysTick_RegDef_t*)SYSTICK_BASEADDR);

#define SYSTICK ((SysTick_RegDef_t*)SYSTICK_BASEADDR)



#endif /* INC_SYSTICK_H_ */
