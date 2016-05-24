/*
 * rgbLed.h
 *
 *  Created on: 19 gen 2016
 *      Author: Emil Kallias
 */

#ifndef PERIPHERAL_INC_RGBLED_H_
#define PERIPHERAL_INC_RGBLED_H_

#define RED 		255,0,0
#define ORANGE 		255,127,0
#define YELLOW 		255,255,0
#define GREENYELLOW 127,255,0
#define GREEN 		0,255,0
#define GREENCYAN 	0,255,127
#define CYAN 		0,255,255
#define BLUECYAN	0,127,255
#define BLUE 		0,0,255
#define BLUEMAGENTA 127,0,255
#define MAGENTA 	255,0,255
#define REDMAGENTA	255,0,127



/* Public function prototypes ------------------------------------------------*/

void SetRGBLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

void RGBLedTest(uint8_t runs);
void RGBLedFade(uint8_t runs);

#endif /* PERIPHERAL_INC_RGBLED_H_ */
