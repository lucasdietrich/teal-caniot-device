#include "test.h"

void test_init(void)
{
#if defined(CONFIG_STM32_MCO)
    /* Should give 80 MHz / 16 = 5 MHz
     */
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16); 
#endif
}