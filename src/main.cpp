
#include "Arduino.h"

volatile uint32_t lastCapture = 0;
volatile uint32_t highTime = 0;
volatile uint32_t lowTime = 0;
volatile bool risingEdge = true;

extern "C" void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC1IF) { // Capture flag yoxlanılır
        uint32_t currentCapture = TIM2->CCR1; // Cari vaxt oxunur

        if (risingEdge) {
            lowTime = currentCapture - lastCapture; // LOW müddəti hesablanır
            TIM2->CCER &= ~TIM_CCER_CC1P; // Növbəti ölçmə üçün yüksələn kənara keçir
        } else {
            highTime = currentCapture - lastCapture; // HIGH müddəti hesablanır
            TIM2->CCER |= TIM_CCER_CC1P; // Növbəti ölçmə üçün enən kənara keçir
        }

        lastCapture = currentCapture;
        risingEdge = !risingEdge; // Növbəti vəziyyət dəyişdirilir

        TIM2->SR &= ~TIM_SR_CC1IF; // Bayraq sıfırlanır
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    }  
    
}

void InitPwm(){
    //pinMode(PA6, OUTPUT);
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    TIM3->PSC = 170 - 1; // 1 µs üçün prescaler (170 MHz / 170 = 1 MHz)
    TIM3->ARR = 20000; // Maksimum dəyər
    TIM3->CCMR1 |= (6UL << TIM_CCMR1_OC2M_Pos) | (6UL << TIM_CCMR1_OC1M_Pos) | (TIM_CCMR1_OC1PE); // CH1 input capture rejimi
    TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E; // Capture aktiv edilir
    //TIM3->DIER |= TIM_DIER_CC2IE;
    TIM3->CCR2 = TIM3->ARR * 20 /100;
    TIM3->CCR1 = TIM3->ARR * 40 /100;
    TIM3->CR1 |= TIM_CR1_CEN; // Timer işə düşür

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIOA->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE6_Msk);  // Clear bits
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODE4_Pos) | (0b10 << GPIO_MODER_MODE6_Pos); 
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL6); //~GPIO_AFRL_AFRL4_Msk;    // Clear AF bits
    GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL4_Pos) | (2 << GPIO_AFRL_AFSEL6_Pos);  // AF2 for TIM3_CH2
}

void setup() {
    Serial.begin(115200);
    //pinMode(PA0, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    // TIM2 aktiv edilir
     __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    //RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->PSC = 170 - 1; // 1 µs üçün prescaler (170 MHz / 170 = 1 MHz)
    TIM2->ARR = 0xffff; // Maksimum dəyər
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; // CH1 input capture rejimi
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;  // Capture aktiv edilir
    TIM2->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE; // Interrupt aktiv edilir
    TIM2->CR1 |= TIM_CR1_CEN; // Timer işə düşür

    GPIOA->MODER &=~GPIO_MODER_MODE0_Msk;
    GPIOA->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOA->AFR[0] &=~GPIO_AFRL_AFRL0;
    GPIOA->AFR[0] |= 1 << GPIO_AFRL_AFSEL0_Pos;
    
    NVIC_EnableIRQ(TIM2_IRQn); // IRQ aktiv edilir
    NVIC_EnableIRQ(TIM3_IRQn); // IRQ aktiv edilir
    InitPwm();
    Serial.println("Start...");
}

void loop() {
    Serial.print("HIGH Time: ");
    Serial.print(highTime);
    Serial.print(" us, LOW Time: ");
    Serial.print(lowTime);
    Serial.println(" us");
    delay(500);
}
