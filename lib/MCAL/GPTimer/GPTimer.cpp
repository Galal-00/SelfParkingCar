#include "GPTimer.h"
#include "GPIO/GPIO.h"
#include "RCC/RCCInterface.h"

void GPTimer::initBasic(TIM_TypeDef *TIMx, uint16_t prescaler, uint32_t arr)
{
    RCCInterface::enableTimer(TIMx);

    TIMx->PSC = prescaler;
    TIMx->ARR = arr;
    TIMx->CNT = 0;
    TIMx->EGR = TIM_EGR_UG;  // <--- Force update
    TIMx->CR1 |= TIM_CR1_CEN;
}

void GPTimer::delayMs(TIM_TypeDef *TIMx, uint32_t ms)
{
    initBasic(TIMx, 8400 - 1, ms * 10 - 1); // 10 kHz, 0.1 ms per count at 84 MHz clock speed

    TIMx->SR = 0;
    while (!(TIMx->SR & TIM_SR_UIF))
        ;
    TIMx->SR = 0;
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

void GPTimer::delayUs(TIM_TypeDef *TIMx, uint32_t us)
{
    initBasic(TIMx, 84 - 1, us - 1); // 1 MHz, 1 us per count at 84 MHz clock speed

    TIMx->SR = 0;
    while (!(TIMx->SR & TIM_SR_UIF))
        ;
    TIMx->SR = 0;
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

uint32_t GPTimer::getCounterValue(TIM_TypeDef *TIMx)
{
    return TIMx->CNT;
}

bool GPTimer::initPwm(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint32_t arr)
{
    // Enable GPIO PWM pin
    if (!enableGpioPwmAf(TIMx, channel))
        return false;

    RCCInterface::enableTimer(TIMx);

    TIMx->PSC = prescaler;
    TIMx->ARR = arr;

    switch (channel)
    {
    case 1:
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
        TIMx->CCMR1 |= (6 << 4); // PWM mode 1
        TIMx->CCMR1 |= TIM_CCMR1_OC1PE;
        TIMx->CCER |= TIM_CCER_CC1E;
        break;
    case 2:
        TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
        TIMx->CCMR1 |= (6 << 12);
        TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
        TIMx->CCER |= TIM_CCER_CC2E;
        break;
    case 3:
        TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
        TIMx->CCMR2 |= (6 << 4);
        TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
        TIMx->CCER |= TIM_CCER_CC3E;
        break;
    case 4:
        TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
        TIMx->CCMR2 |= (6 << 12);
        TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
        TIMx->CCER |= TIM_CCER_CC4E;
        break;
    }

    TIMx->CR1 |= TIM_CR1_ARPE;
    TIMx->EGR |= TIM_EGR_UG;
    TIMx->CR1 |= TIM_CR1_CEN;

    return true;
}

void GPTimer::setPwmDuty(TIM_TypeDef *TIMx, uint8_t channel, float dutyPercentage)
{
    // Calculate CCR value
    uint32_t mapped_duty = static_cast<uint32_t>(((dutyPercentage / 100.0f) * (static_cast<uint32_t>(TIMx->ARR) + 1)));

    switch (channel)
    {
    case 1:
        TIMx->CCR1 = mapped_duty;
        break;
    case 2:
        TIMx->CCR2 = mapped_duty;
        break;
    case 3:
        TIMx->CCR3 = mapped_duty;
        break;
    case 4:
        TIMx->CCR4 = mapped_duty;
        break;
    }

    // Generate update event
    TIMx->EGR |= TIM_EGR_UG;
}

bool GPTimer::enableGpioPwmAf(TIM_TypeDef *TIMx, uint8_t channel)
{
    // Check for invalid channel value
    if (channel <= 0 || channel > 4)
        return false;

    if (TIMx == TIM2)
    {
        // Pin A0 -> A3, AF 1
        GPIO pwmPin(GPIOA, channel - 1);
        pwmPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 1);
        // Keep it in case we want to become modular
        switch (channel)
        {
        case 1:
            /* code */
            break;
        case 2:
            /* code */
            break;
        case 3:
            /* code */
            break;
        case 4:
            /* code */
            break;
        default:
            break;
        }
    }
    else if (TIMx == TIM3)
    {
        if (channel == 1)
        {
            // Pin B4, AF 2
            GPIO pwmPin1(GPIOB, 4);
            pwmPin1.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
        }
        else if (channel == 2)
        {
            // Pin B5, AF 2
            GPIO pwmPin2(GPIOB, 5);
            pwmPin2.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
        }
        else if (channel == 3)
        {
            // Pin B0, AF 2
            GPIO pwmPin3(GPIOB, 0);
            pwmPin3.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
        }
        else if (channel == 4)
        {
            // Pin B1, AF 2
            GPIO pwmPin4(GPIOB, 1);
            pwmPin4.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
        }
    }
    else if (TIMx == TIM4)
    {
        // TIM4 channels have only one possible pin each
        // Pin B6 -> B9, AF 2
        GPIO pwmPin(GPIOB, channel + 5);
        pwmPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
    }
    else if (TIMx == TIM5)
    {
        // Pin A0 -> A3, AF 2
        GPIO pwmPin(GPIOA, channel - 1);
        pwmPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::High, 2);
    }
    else
        return false;

    return true;
}