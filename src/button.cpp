#include "button.h"

void ButtonSetup(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = BTTEAM_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTTEAM_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BTTIRETTE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTTIRETTE_Port, &GPIO_InitStruct);
}
GPIO_PinState ButtonTeamGetValue(void)
{
    return HAL_GPIO_ReadPin(BTTEAM_Port, BTTEAM_pin);
}
GPIO_PinState ButtonTiretteGetValue(void)
{
    return HAL_GPIO_ReadPin(BTTIRETTE_Port, BTTIRETTE_Pin);
}
