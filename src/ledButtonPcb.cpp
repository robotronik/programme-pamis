#include "ledButtonPcb.h"


void * isr_buttonpcb_fallbak(void); 

void LedPcbSetup(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*Configure GPIO pins : SW_USER_Pin SW_TEAM_Pin */
    GPIO_InitStruct.Pin = LEDPCB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LEDPCB_Port, &GPIO_InitStruct);
}

void LedPcbToggle(void){
    HAL_GPIO_TogglePin(LEDPCB_Port,LEDPCB_Pin);
}
void LedPcbOn(void){
    HAL_GPIO_WritePin(LEDPCB_Port,LEDPCB_Pin, GPIO_PIN_SET); 
}
void LedPcbOff(void){
    HAL_GPIO_WritePin(LEDPCB_Port,LEDPCB_Pin, GPIO_PIN_RESET); 
}

void ButtonPcbSetup(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = BUTTONPCB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTONPCB_Port, &GPIO_InitStruct);
}
void ButtonPcbSetupInterrupt(void *isr(void)){
    //todo 
}
GPIO_PinState ButtonPcbGetValue(void){
    return HAL_GPIO_ReadPin(BUTTONPCB_Port,BUTTONPCB_Pin);
}
void ButtonPcbEnableInterrupt(void){
    //TODO
}
void ButtonPcbDisableInterrupt(void){
    //TODO
}
