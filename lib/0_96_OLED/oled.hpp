
#include "Arduino.h"
void OLED_Init(void);
void OLED_DrawBMP(unsigned char x,unsigned char y,unsigned char sizex, unsigned char sizey,const unsigned char BMP[]);
void OLED_ShowChinese(unsigned char x,unsigned char y,const unsigned char no,unsigned char sizey);
void OLED_ShowString(unsigned char x,unsigned char y,const char *chr,unsigned char sizey);
void OLED_ShowNum(unsigned char x,unsigned char y,unsigned int num,unsigned char len,unsigned char sizey);
void OLED_ShowChar(unsigned char x,unsigned char y,const unsigned char chr,unsigned char sizey);
void OLED_Clear(void)  ;
void OLED_Display_Off(void);
void OLED_Display_On(void);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_DisplayTurn(unsigned char i);
void OLED_ColorTurn(unsigned char i);
void OLED_init1(void);