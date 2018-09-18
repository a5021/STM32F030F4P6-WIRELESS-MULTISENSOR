#ifndef __GPIO_INCLUDED
#define __GPIO_INCLUDED

#define ANALOG_MODE_FOR_ALL_PINS      0xFFFFFFFF
#define PIN_CONF(PIN, MODE)           ((uint32_t)MODE << (PIN * 2))
#define PIN_AF(PIN, AF)               ((uint32_t)AF << (4 * (PIN & 7)))
      
#define PIN(PIN_NO)                   (PIN_NO)
#define AF(PIN_NO)                    (PIN_NO)

#define PIN_MODE_INPUT                0
#define PIN_MODE_OUTPUT               1
#define PIN_MODE_ALT_FUNC             2
#define PIN_MODE_ANALOG               3
                                      
#define PINV_INPUT                    3
#define PINV_OUTPUT                   2
#define PINV_ALT_FUNC                 1
#define PINV_ANALOG                   0
                                      
#define PIN_SPEED_LOW                 0
#define PIN_SPEED_MEDIUM              1
#define PIN_SPEED_HIGH                3
                                      
#define PIN_PULL_NO                   0
#define PIN_PULL_UP                   1
#define PIN_PULL_DOWN                 2

#define LOW                           GPIO_BSRR_BR_
#define HIGH                          GPIO_BSRR_BS_

#define CONCATENATE(A, B)             A ## B
#define GLUE(A, B)                    CONCATENATE(A, B)
#define TOGGLE_PIN(PORT, PIN, STATE)  GPIO ## PORT->BSRR = CONCATENATE(STATE, PIN)

#endif
