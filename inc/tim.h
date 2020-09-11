#ifndef __TIM_INCLUDED
#define __TIM_INCLUDED

#include "control.h"

#define INIT_TIMER(P, A)                           \
          RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;     \
          TIM14->CR1 = TIM_CR1_URS;                \
          TIM14->PSC = (P) - 1;                    \
          TIM14->ARR = (A) - 1;                    \
          TIM14->EGR = TIM_EGR_UG;                 \
          TIM14->DIER = TIM_DIER_UIE;              \
          TIM14->SR = 0;                           \
          NVIC_ClearPendingIRQ(TIM14_IRQn)
          
#define RUN_TIMER()  TIM14->CR1 = TIM_CR1_CEN

#define S_DELAY(DIV) do {                          \
          INIT_TIMER(DIV, 2);                      \
          uint32_t rcc_cfgr = RCC->CFGR;           \
          RUN_MCU_AT(LOWEST_FREQ);                 \
          RUN_TIMER();                             \
          __WFE();                                 \
          RCC->CFGR = rcc_cfgr;                    \
          RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;   \
          RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;    \
          RCC->APB1RSTR = 0;                       \
        } while(0)

__STATIC_INLINE void s_delay(uint32_t div) {
  
  INIT_TIMER(div, 2);            
  uint32_t rcc_cfgr = RCC->CFGR;        
  RUN_MCU_AT(LOWEST_FREQ);
  RUN_TIMER();
  
  __WFE();                              
  
  RCC->CFGR = rcc_cfgr;                 
  RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN; 
  RCC->APB1RSTR = 0;                    
}

#endif
