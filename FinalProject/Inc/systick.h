/**
 *
 */

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include <stdint.h>

extern volatile uint8_t systick;

void systick_init(void);


#endif /* _SYSTICK_H_ */
