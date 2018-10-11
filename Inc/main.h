#ifndef __MAIN_H
#define __MAIN_H

#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOB
#define LD1_ON      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LD2_ON      GPIOB->ODR &= ~(1 << 14)
#define LD3_ON      GPIOB->ODR &= ~(1 << 15) 
#define LD1_OFF     GPIOB->ODR |= (1 << 13)
#define LD2_OFF     GPIOB->ODR |= (1 << 14)
#define LD3_OFF     GPIOB->ODR |= (1 << 15)
#define LD1_TOGGLE  GPIOB->ODR ^= (1 << 13)
#define LD2_TOGGLE  GPIOB->ODR ^= (1 << 14)
#define LD3_TOGGLE  GPIOB->ODR ^= (1 << 15)

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)


#endif

