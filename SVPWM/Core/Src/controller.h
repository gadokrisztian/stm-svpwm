#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

#include <stdint.h>

// tim uptade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1)

typedef struct {

	uint32_t sysclk;

} Controller;


static Controller ctrl;

void controller_init(uint32_t sysclk);


#endif /* SRC_CONTROLLER_H_ */
