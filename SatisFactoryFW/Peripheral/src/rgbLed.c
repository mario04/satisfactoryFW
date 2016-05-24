/*
 * rgbLed.c
 *
 *  Created on: 19 gen 2016
 *      Author: Emil Kallias
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

#include "rgbLed.h"

/* Private defines -----------------------------------------------------------*/

#define RGBTEST_DELAY 		100
#define RGBTEST_BRIGHTNESS	50

/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;

uint16_t rgb_lookup[] = {
		0x0000, 0x1FFF, 0x32B7, 0x3FFF, 0x4A4C, 0x52B7, 0x59D5, 0x5FFF,
		0x656F, 0x6A4C, 0x6EB3, 0x72B7, 0x7669, 0x79D5, 0x7D04, 0x7FFF,
		0x82CB, 0x856F, 0x87EE, 0x8A4C,	0x8C8D, 0x8EB3, 0x90C0, 0x92B7,
		0x9499, 0x9669, 0x9827, 0x99D5, 0x9B73, 0x9D04,	0x9E88, 0x9FFF,
		0xA16B, 0xA2CB, 0xA422, 0xA56F, 0xA6B3, 0xA7EE, 0xA921, 0xAA4C,
		0xAB70, 0xAC8D, 0xADA3, 0xAEB2, 0xAFBC, 0xB0C0, 0xB1BE, 0xB2B7,
		0xB3AB, 0xB499,	0xB583, 0xB669, 0xB74A, 0xB827, 0xB900, 0xB9D5,
		0xBAA6, 0xBB73, 0xBC3D, 0xBD04,	0xBDC7, 0xBE88, 0xBF45, 0xBFFF,
		0xC0B6, 0xC16A, 0xC21C, 0xC2CB, 0xC378, 0xC422,	0xC4C9, 0xC56F,
		0xC612, 0xC6B3, 0xC751, 0xC7EE, 0xC888, 0xC921, 0xC9B7, 0xCA4C,
		0xCADF, 0xCB70, 0xCBFF, 0xCC8D, 0xCD18, 0xCDA3, 0xCE2B, 0xCEB2,
		0xCF38, 0xCFBC,	0xD03F, 0xD0C0, 0xD13F, 0xD1BE, 0xD23B, 0xD2B7,
		0xD331, 0xD3AA, 0xD422, 0xD499, 0xD50F, 0xD583, 0xD5F6, 0xD669,
		0xD6DA, 0xD74A, 0xD7B9, 0xD827, 0xD894, 0xD900,	0xD96B, 0xD9D5,
		0xDA3E, 0xDAA6, 0xDB0D, 0xDB73, 0xDBD9, 0xDC3D, 0xDCA1, 0xDD04,
		0xDD66, 0xDDC7, 0xDE28, 0xDE87, 0xDEE6, 0xDF45, 0xDFA2, 0xDFFF,
		0xE05B, 0xE0B6,	0xE110, 0xE16A, 0xE1C3, 0xE21C, 0xE274, 0xE2CB,
		0xE322, 0xE378, 0xE3CD, 0xE422,	0xE476, 0xE4C9, 0xE51C, 0xE56F,
		0xE5C0, 0xE612, 0xE662, 0xE6B2, 0xE702, 0xE751,	0xE7A0, 0xE7EE,
		0xE83B, 0xE888, 0xE8D5, 0xE921, 0xE96C, 0xE9B7, 0xEA02, 0xEA4C,
		0xEA95, 0xEADF, 0xEB27, 0xEB70, 0xEBB7, 0xEBFF, 0xEC46, 0xEC8C,
		0xECD3, 0xED18,	0xED5E, 0xEDA3, 0xEDE7, 0xEE2B, 0xEE6F, 0xEEB2,
		0xEEF5, 0xEF38, 0xEF7A, 0xEFBC,	0xEFFD, 0xF03E, 0xF07F, 0xF0C0,
		0xF100, 0xF13F, 0xF17F, 0xF1BE, 0xF1FC, 0xF23B,	0xF279, 0xF2B7,
		0xF2F4, 0xF331, 0xF36E, 0xF3AA, 0xF3E6, 0xF422, 0xF45E, 0xF499,
		0xF4D4, 0xF50F, 0xF549, 0xF583, 0xF5BD, 0xF5F6, 0xF630, 0xF669,
		0xF6A1, 0xF6DA,	0xF712, 0xF74A, 0xF781, 0xF7B9, 0xF7F0, 0xF827,
		0xF85D, 0xF893, 0xF8CA, 0xF8FF,	0xF935, 0xF96A, 0xF99F, 0xF9D4,
		0xFA09, 0xFA3D, 0xFA72, 0xFAA6, 0xFAD9, 0xFB0D,	0xFB40, 0xFB73,
		0xFBA6, 0xFBD9, 0xFC0B, 0xFC3D, 0xFC6F, 0xFCA1, 0xFCD2, 0xFD04,
		0xFD35, 0xFD66, 0xFD97, 0xFDC7, 0xFDF7, 0xFE28, 0xFE58, 0xFE87,
		0xFEB7, 0xFEE6,	0xFF15, 0xFF44, 0xFF73, 0xFFA2, 0xFFD0, 0xFFFF
};

/* Private function prototypes -----------------------------------------------*/



/* Public function implementation --------------------------------------------*/


void SetRGBLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
	static TIM_OC_InitTypeDef sConfigOC;
	//float rSet;
	//float brightnessSet;

	//brightnessSet = (((float)(brightness%100))/100);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

	//rSet = (255-(r*(((float)(brightness%100))/100)));
	sConfigOC.Pulse = rgb_lookup[(uint8_t)(255-(r*(((float)(brightness%101))/100)))];
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

	sConfigOC.Pulse = rgb_lookup[(uint8_t)(255-(g*(((float)(brightness%101))/100)))];
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

	sConfigOC.Pulse = rgb_lookup[(uint8_t)(255-(b*(((float)(brightness%101))/100)))];
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


}


void RGBLedTest(uint8_t runs)
{


	for(uint8_t i = 0; i < runs; i++ )
	{
		SetRGBLed(RED, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(ORANGE, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(YELLOW, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(GREENYELLOW, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(GREEN, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(GREENCYAN, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(CYAN, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(BLUECYAN, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(BLUE, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(BLUEMAGENTA, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(MAGENTA, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);
		SetRGBLed(REDMAGENTA, RGBTEST_BRIGHTNESS);
		HAL_Delay(RGBTEST_DELAY);


	}


}

void RGBLedFade(uint8_t runs)
{
	uint8_t r = 255, g = 0, b = 0;

	for(uint8_t i = 0; i < runs; i++ )
	{
		for(g=0; g < 255; g++){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(2);
		}
		for(r=255; r > 0; r--){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(2);
		}

		for(b=0; b < 255; b++){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(2);
		}
		for(g=255; g > 0; g--){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(2);
		}

		for(r=0; r < 255; r++){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(1);
		}
		for(b=255; b > 0; b--){
			SetRGBLed(r, g, b, RGBTEST_BRIGHTNESS);
			HAL_Delay(2);
		}
	}

}


/* Private function implementation -------------------------------------------*/
