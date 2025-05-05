#include "RCCInterface.h"

#include "SystemTimer/SystemTimer.h"

constexpr float HSE_VALUE_MHZ = 25.0f;

bool RCCInterface::validatePLL(uint8_t pllm, uint16_t plln, uint8_t pllp)
{
    if (pllm < 2 || pllm > 63)
        return false;
    if (plln < 192 || plln > 432)
        return false;
    if (pllp != 2 && pllp != 4 && pllp != 6 && pllp != 8)
        return false;

    float vco_in = HSE_VALUE_MHZ / pllm;
    float vco_out = vco_in * plln;
    float sysclk = vco_out / pllp;

    return (vco_in >= 1.0f && vco_in <= 2.0f) &&
           (vco_out >= 192.0f && vco_out <= 432.0f) &&
           (sysclk <= 100.0f);
}

bool RCCInterface::initSystemClock(uint8_t PLLM, uint16_t PLLN, uint8_t PLLP)
{
    // Validate PLL values
    if (!validatePLL(PLLM, PLLN, PLLP))
        return false;

    // FLASH settings for 84 MHz at > 2.7 V
    FLASH->ACR |=
        FLASH_ACR_DCEN |       // enable data cache
        FLASH_ACR_ICEN |       // enable instruction cache
        FLASH_ACR_PRFTEN |     // enable prefetch
        FLASH_ACR_LATENCY_2WS; // 2 wait states

    // Assuming HSE is used and PLL configured for 84 MHz
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY))
        ; // Wait till HSE is ready

    // Clear PLLM, PLLN, PLLP, PLLQ bits
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk);

    // PLL Clock Rate Equation (Hz) = (f_HSE / PLLM) * PLLN / PLLP
    RCC->PLLCFGR = (RCC_PLLCFGR_PLLSRC_HSE | // Use HSE as PLL source
                    (PLLM << RCC_PLLCFGR_PLLM_Pos) |
                    (PLLN << RCC_PLLCFGR_PLLN_Pos) |
                    ((PLLP / 2 - 1) << RCC_PLLCFGR_PLLP_Pos));

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // Ensure APB1 max clock rate is 50MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // No div for APB2

    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ; // Wait till PLL is ready

    RCC->CFGR |= RCC_CFGR_SW_PLL; // Select PLL as system clock
    while (!(RCC->CFGR & RCC_CFGR_SWS))
        ;

    // Disable internal HSI as it is unneeded
    RCC->CR &= ~(RCC_CR_HSION_Msk);

    // Set SystemCoreClock variable
    SystemCoreClock = 84000000;

    SystemTimer::init();

    return true;
}

bool RCCInterface::enableGPIO(GPIO_TypeDef *gpio)
{
    if (gpio == GPIOA)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (gpio == GPIOB)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (gpio == GPIOC)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (gpio == GPIOD)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (gpio == GPIOE)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else
        return false;

    return true;
}

bool RCCInterface::enableTimer(TIM_TypeDef *timer)
{
    if (timer == TIM1)
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (timer == TIM2)
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (timer == TIM3)
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (timer == TIM4)
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (timer == TIM5)
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    else
        return false;

    return true;
}

bool RCCInterface::enableUSART(USART_TypeDef *USARTx)
{
    if (USARTx == USART1)
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    else if (USARTx == USART2)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    else if (USARTx == USART6)
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    else
        return false;

    return true;
}