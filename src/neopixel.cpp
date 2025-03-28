#include "neopixel.h"

#define NEOPIXEL_RESET_TIMER_TICK 64
#define PWM_HI 70
#define PWM_LOW 35

#define NB_BYTES ((NEOPIXEL_NB + 1) * 24 + NEOPIXEL_RESET_TIMER_TICK)

TIM_HandleTypeDef htimneopixel;
DMA_HandleTypeDef hdma_timneopixel_ch1;

volatile uint8_t datasentflag;

uint32_t data[NB_BYTES];

void neopixelSetup(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htimneopixel.Instance = TIM2;
    htimneopixel.Init.Prescaler = 0;
    htimneopixel.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimneopixel.Init.Period = 104; // Fréquence WS2812 (~800kHz)
    htimneopixel.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimneopixel.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htimneopixel) != HAL_OK)
        Error_Handler();

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_Init(&htimneopixel) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htimneopixel, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_timneopixel_ch1.Instance = DMA1_Channel1;
    hdma_timneopixel_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_timneopixel_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_timneopixel_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_timneopixel_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_timneopixel_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_timneopixel_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_timneopixel_ch1.Init.Mode = DMA_NORMAL;
    hdma_timneopixel_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_timneopixel_ch1) != HAL_OK)
        Error_Handler();
    __HAL_LINKDMA(&htimneopixel, hdma[TIM_DMA_ID_CC1], hdma_timneopixel_ch1);

    __HAL_DMA_ENABLE(&hdma_timneopixel_ch1);
    __HAL_TIM_ENABLE_DMA(&htimneopixel, TIM_DMA_CC1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_TIM_PWM_Start(&htimneopixel, TIM_CHANNEL_1);

    HAL_TIM_PWM_Stop_DMA(&htimneopixel, TIM_CHANNEL_1);
    HAL_DMA_Abort(&hdma_timneopixel_ch1);

    datasentflag = 1;
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
    {
        datasentflag = 1;
        HAL_TIM_PWM_Stop_DMA(&htimneopixel, TIM_CHANNEL_1);
    }
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_timneopixel_ch1);
}

static void _sendNeopixel(void)
{
    for (int i = (NEOPIXEL_NB + 1) * 24; i < (NEOPIXEL_NB + 1) * 24 + NEOPIXEL_RESET_TIMER_TICK; i++)
    {
        data[i] = 0;
    }
    uint32_t timeout = HAL_GetTick();
    while (!datasentflag)
    {
        if (HAL_GetTick() - timeout > 100) // Timeout de 100 ms
        {
            Error_Handler();
            break;
        }
    }
    datasentflag = 0;

    if (htimneopixel.State != HAL_TIM_STATE_READY)
    {
        uartprintf("TIM2 n'est pas prêt !\n");
        Error_Handler();
    }
    if (hdma_timneopixel_ch1.State != HAL_DMA_STATE_READY)
    {
        uartprintf("DMA1_Channel1 n'est pas prêt !\n");
        Error_Handler();
    }
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(&htimneopixel, TIM_CHANNEL_1, data, NB_BYTES);

    if (status != HAL_OK)
    {
        if (status == HAL_ERROR)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_ERROR\n");
        else if (status == HAL_BUSY)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_BUSY\n");
        else if (status == HAL_TIMEOUT)
            uartprintf("Erreur: HAL_TIM_PWM_Start_DMA() a retourné HAL_TIMEOUT\n");

        Error_Handler();
    }
}

void neopixelClear(void)
{
    for (int i = 0; i < (NEOPIXEL_NB + 1) * 24; i++)
    {
        data[i] = PWM_LOW;
    }
    _sendNeopixel();
}

void neopixelSetLed(uint16_t numLed, colorRGB_t color)
{
    neopixelSetMoreLeds(numLed, &color, 1);
}

void neopixelSetLed(uint16_t numLed, colorRGBA_t color)
{
    colorRGB_t colorRGB = RGBA_to_RGB(color);
    neopixelSetMoreLeds(numLed, &colorRGB, 1);
}
void neopixelSetLed(uint16_t numLed, colorHSV_t color)
{
    colorRGB_t colorRGB = HSV_to_RGB(color);
    neopixelSetMoreLeds(numLed, &colorRGB, 1);
}

void __writeData(uint16_t indx, uint32_t col)
{
    indx = indx * 24;
    for (int i = 23; i >= 0; i--)
    {
        if (col & (1 << i))
        {
            data[indx] = PWM_HI;
        }

        else
        {
            data[indx] = PWM_LOW;
        }
        indx++;
    }
}

void neopixelSetMoreLeds(uint16_t firstled, colorRGB_t color[], size_t size)
{
    for (uint16_t j = 0; j < size; j++)
    {
        uint32_t col = color[j].green << 16 | color[j].red << 8 | color[j].blue;
        __writeData(j + firstled, col);
    }
    _sendNeopixel();
}
void neopixelSetMoreLeds(uint16_t firstled, colorHSV_t color[], size_t size)
{
    for (uint16_t j = 0; j < size; j++)
    {
        colorRGB_t RGB = HSV_to_RGB(color[j]);
        uint32_t col = RGB.green << 16 | RGB.red << 8 | RGB.blue;
        __writeData(j + firstled, col);
    }
    _sendNeopixel();
}
void neopixelSetMoreLeds(uint16_t firstled, colorRGBA_t color[], size_t size)
{
    for (uint16_t j = 0; j < size; j++)
    {
        colorRGB_t RGB = RGBA_to_RGB(color[j]);
        uint32_t col = RGB.green << 16 | RGB.red << 8 | RGB.blue;
        __writeData(j + firstled, col);
    }
    _sendNeopixel();
}
/**
 * @brief Convertit une couleur HSV en RGB
 */
colorRGB_t HSV_to_RGB(colorHSV_t hsv)
{
    float h = hsv.H, s = hsv.S, v = hsv.V;
    float c = v * s;
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;

    float r1, g1, b1;
    if (h < 60)
    {
        r1 = c;
        g1 = x;
        b1 = 0;
    }
    else if (h < 120)
    {
        r1 = x;
        g1 = c;
        b1 = 0;
    }
    else if (h < 180)
    {
        r1 = 0;
        g1 = c;
        b1 = x;
    }
    else if (h < 240)
    {
        r1 = 0;
        g1 = x;
        b1 = c;
    }
    else if (h < 300)
    {
        r1 = x;
        g1 = 0;
        b1 = c;
    }
    else
    {
        r1 = c;
        g1 = 0;
        b1 = x;
    }

    colorRGB_t rgb = {
        .red = (uint8_t)((r1 + m) * 255),
        .green = (uint8_t)((g1 + m) * 255),
        .blue = (uint8_t)((b1 + m) * 255)};
    return rgb;
}

/**
 * @brief Convertit une couleur HSV en RGBA (Alpha fixe à 255)
 */
colorRGBA_t HSV_to_RGBA(colorHSV_t hsv)
{
    colorRGB_t rgb = HSV_to_RGB(hsv);
    colorRGBA_t rgba = {rgb.red, rgb.green, rgb.blue, 255};
    return rgba;
}

/**
 * @brief Convertit une couleur RGB en HSV
 */
colorHSV_t RGB_to_HSV(colorRGB_t rgb)
{
    float r = rgb.red / 255.0;
    float g = rgb.green / 255.0;
    float b = rgb.blue / 255.0;

    float max = fmaxf(r, fmaxf(g, b));
    float min = fminf(r, fminf(g, b));
    float delta = max - min;

    float h = 0.0, s = 0.0, v = max;

    if (delta > 0.00001)
    {
        if (max == r)
        {
            h = 60 * fmod(((g - b) / delta), 6);
        }
        else if (max == g)
        {
            h = 60 * (((b - r) / delta) + 2);
        }
        else if (max == b)
        {
            h = 60 * (((r - g) / delta) + 4);
        }

        if (h < 0)
            h += 360;
    }

    s = (max == 0) ? 0 : (delta / max);

    colorHSV_t hsv = {h, s, v};
    return hsv;
}
colorRGB_t RGBA_to_RGB(colorRGBA_t rgba)
{
    float brightness = rgba.alpha / 255.0; // Normalisation de la luminosité (0.0 - 1.0)

    colorRGB_t rgb = {
        .red = (uint8_t)(rgba.red * brightness),
        .green = (uint8_t)(rgba.green * brightness),
        .blue = (uint8_t)(rgba.blue * brightness)};

    return rgb;
}
