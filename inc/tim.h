#ifndef __TIM_INCLUDED
#define __TIM_INCLUDED

#include "control.h"

#define INIT_TIMER(TIM, P, A)                      \
          if (TIM == TIM2) {                       \
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    \
          } else if (TIM == TIM3) {                \
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;    \
          } else if (TIM == TIM14){                \
            RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;   \
          } else if (TIM == TIM1) {                \
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;    \
          } else if (TIM == TIM16) {               \
            RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;   \
          } else if (TIM == TIM17) {               \
            RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;   \
          }                                        \
          TIM->CR1 = TIM_CR1_URS;                  \
          TIM->PSC = (P) - 1;                      \
          TIM->ARR = (A) - 1;                      \
          TIM->EGR = TIM_EGR_UG;                   \
          TIM->DIER = TIM_DIER_UIE;                \
          TIM->SR = 0;                             \
          NVIC_ClearPendingIRQ(TIM##_IRQn);
          
#define RUN_TIMER(TIM)  TIM->CR1 = TIM_CR1_CEN

#define S_DELAY(DIV) do {                          \
          INIT_TIMER(TIM14, DIV, 2);               \
          uint32_t rcc_cfgr = RCC->CFGR;           \
          RUN_MCU_AT(LOWEST_FREQ);                 \
          RUN_TIMER(TIM14);                        \
          __WFE();                                 \
          RCC->CFGR = rcc_cfgr;                    \
          RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;   \
          RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;    \
          RCC->APB1RSTR = 0;                       \
        } while(0)
          
#define DELAY(DIV, CNT)                            \
          INIT_TIMER(TIM14, 8 * DIV, CNT);         \
          RUN_TIMER(TIM14);                        \
          __WFE();                                 \
          RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;   \
          RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;    \
          RCC->APB1RSTR = 0
          
#define DELAY_US(US)                               \
          INIT_TIMER(TIM14, 8, US);                \
          RUN_TIMER(TIM14);                        \
          __WFE();                                 \
          RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;   \
          RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;    \
          RCC->APB1RSTR = 0
                      
#define DELAY_MS(MS)                               \
          INIT_TIMER(TIM14, 8000, MS);             \
          RUN_TIMER(TIM14);                        \
          __WFE();                                 \
          RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;   \
          RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;    \
          RCC->APB1RSTR = 0

__STATIC_INLINE void s_delay(uint32_t div) {
  INIT_TIMER(TIM14, div, 2);            
  uint32_t rcc_cfgr = RCC->CFGR;        
  RUN_MCU_AT(LOWEST_FREQ);
  RUN_TIMER(TIM14);
  __WFE();                              
  RCC->CFGR = rcc_cfgr;                 
  RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN; 
  RCC->APB1RSTR = 0;                    
}

#endif
