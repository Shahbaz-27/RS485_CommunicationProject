
#include "main.h"
#include "stdio.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


#define LCD_RS	11
#define LCD_EN	12



void GPIO_init();
void UART_init();


void lcd_init(void); 
void lcd_sendCommand(unsigned char cmd);
void lcd_sendData (unsigned char data);
void lcd_putValue (unsigned char value);
void lcd_print( char * str);
void lcd_printInt(int number);
void lcd_printInt8(volatile uint8_t number);
void lcd_printFloat(float number, int decimalPoints);


void keypad_checkColumnState();
void keypad_getkey();
//void read_number();
void senddata(uint8_t number);


int flag;
uint8_t whichkey;
/*
uint8_t numeric_value=5;

uint8_t digitcount=0;
uint8_t seconddigit=0;
uint8_t firstdigit=0;
uint8_t loopcount=0;
*/


int main(void)
	
{
	HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	
	/*Bitwise Initializations*/
	GPIO_init();
	UART_init();
	lcd_init();
	HAL_Delay(1000);
	lcd_sendCommand(0x01);
	lcd_print("RS485 Communication Project");
	HAL_Delay(2000);
	lcd_sendCommand(0x01);// Clear the LCD
 	lcd_print("The key is:");
	lcd_sendCommand(0xC0); // Force the cursor to jump to beginning of 2nd line
	HAL_Delay(10);
	lcd_print("LED to be ON:");	
	

	while(1)
	{
		 keypad_checkColumnState();
		
	if(flag==1)
	{
			keypad_getkey();
			lcd_sendCommand(0x8B);// Force the cursor to particular position of 1st line
			HAL_Delay(10);		
			lcd_printInt8(whichkey);
		 	lcd_sendCommand(0xC0);
		  lcd_sendCommand(0xCD);// Force the cursor to particular position of 2nd line
			HAL_Delay(10);			
			lcd_printInt8(whichkey);
			senddata(whichkey);	
			flag=0;
			HAL_Delay(200);
	}
	else 
	{
			GPIOC->ODR ^= (1<<14);
	}   
}
}


/*
void read_number()
{
	
	if(digitcount==1)
	{
		seconddigit = whichkey;
	}
	else if(digitcount==0)
	{
		firstdigit= whichkey;
		digitcount++;
	}
	else
	{	
		GPIOC->ODR ^=(1<<13);
	}
	loopcount++;
	if (loopcount==2)
	{
		number = firstdigit * 10 + seconddigit; 
		lcd_sendCommand(0x01);
		lcd_print("The Number is:");
		lcd_printInt8(number);
		senddata(number);
		HAL_Delay(500);
	}
	else
	{	
		GPIOC->ODR ^=(1<<13);
	}
	
}
*/

void senddata(uint8_t number)
{
	USART1->DR = number;
	while((USART1->SR&(1<<6)) == 0);//wait until TC Flag is HIGH
	USART1->SR &= ~(1<<6); // Clear the TC Flag
}

void UART_init()
{
		RCC->APB2ENR |= (1<<14);
		GPIOA->ODR   |= (1<<10); /*Pull-up RX of USART1*/
		GPIOA->CRH    = 0x444338B4; /* EN and RS Pins as outputs + and UART Pins */
		USART1->CR1   = 0x200C;//
		USART1->BRR   = 7500; /*for B.rate of 9600*/
	
}


void GPIO_init()
{

		RCC->APB2ENR |= 0xFC; /* Enable clocks for GPIO ports */
		GPIOC->CRH    = 0x43344444; // For testing LEDs
		GPIOA->CRL    = 0x33338888; /* PA0-PA3 as inputs and PA4-P47 as outputs for HEX KEYPAD */ 
		GPIOA->ODR   |= 0x0F;			/*Pull-Up GPIO_Inputs A0-A3*/
		GPIOB->CRH    = 0x33334444;/* D4-D7 on PB12-PB15 as GPO @50MHz*/

}




/*________Keypad Intilizations________*/
void keypad_checkColumnState()
{
	// so to make all the rows zero a4-a7
		GPIOA->BRR = 0xF0;
		HAL_Delay(1);
	
if(((GPIOA->IDR)&0xF) != 0xF) 
	// checking that if any column goes downa or n
{
		HAL_Delay(30); 
		flag=1;
}
else
		flag=0; // Means a column goes low but we don't which
}



/* The function returns the pressed key. */
/* It returns 0 if no key is pressed. */
void keypad_getkey()
{
uint8_t keypadLookup[16]=
	{1,2,3,3,4,5,6,6,7,8,9,9,0,0,0};
			const uint32_t rowSelect[4]={
			0x001000E0, /* Row3-Row0 = 1110 PA4LOW */ 
			0x002000D0, /* Row3-Row0 = 1101 PA5LOW*/
			0x004000B0, /* Row3-Row0 = 1011 PA6LOW*/
			0x00800070};/* Row3-Row0 = 0111 PB7LOW*/
			
for(int r = 0; r <= 3; r++) /* rows 0 to 3 */
	{
		GPIOA->BSRR = rowSelect[r]; /* ground row r and make the others high */
		HAL_Delay(1); /* wait for the columns to be updated */
		uint8_t cols = (GPIOA->IDR)&0x0F; /*Reading the state of Coloumns PA0-PA3*/
	switch(cols) 
		{
	case 0x0E: 
		whichkey= keypadLookup[r*4+0]; /* col0 is low */
		break;
	
	case 0x0D:
		whichkey= keypadLookup[r*4+1]; /* col1 is low */
		break;
	
	case 0x0B: 
		whichkey= keypadLookup[r*4+2]; /* col2 is low */
		break;

	case 0x07: 
		whichkey= keypadLookup[r*4+3]; 
		break;/* col3 is low */
		}
	}	
} 




void lcd_putValue(unsigned char value)
{
	
		GPIOB->BRR  = 0xF000;/*cLEAR PB12-PB15*/
		GPIOB->ODR = (((value>>4)& 0xF) << 12);// Logical shift right command to place the upper nibble in ODR
		GPIOA->BRR= (1<<LCD_EN); // L
		HAL_Delay(5);
		GPIOA->BSRR= (1<<LCD_EN); // H
		HAL_Delay(10);
		GPIOA->BRR= (1<<LCD_EN); // L
		HAL_Delay(50);
		GPIOB->BRR  = 0xF000;
		GPIOB->ODR = ((value & 0xF) << 12);// to place the lower nibble in ODR
		GPIOA->BRR= (1<<LCD_EN); // L
		HAL_Delay(5);
		GPIOA->BSRR= (1<<LCD_EN); // H
		HAL_Delay(10);
		GPIOA->BRR= (1<<LCD_EN); // L
		HAL_Delay(50);
	
}



void lcd_print( char * str)
{
	unsigned char i=0;
	while(str[i]!=0)
	{
		lcd_sendData(str[i]);
			if (i==15)
			{
				lcd_sendCommand(0xC0);
			}
		i++;
	}
}




void lcd_printInt(int number)
{
		char numStr[16];
		sprintf(numStr,"%d", number);
		lcd_print(numStr);
}



void lcd_printInt8(volatile uint8_t number)
{
		char numStr[16];
		sprintf(numStr,"%d", number);
		lcd_print(numStr);
} 


void lcd_printFloat(float number, int decimalPoints)
{
		char numStr[16];
		sprintf(numStr,"%.*f",decimalPoints, number);
		lcd_print(numStr);
}




void lcd_sendCommand(unsigned char cmd)
{
		//for sending command first RS=0
		GPIOA->BRR=(1<<LCD_RS);
		GPIOB->BRR = 0xF000; // clearing the data pins 
		lcd_putValue(cmd);

}


void lcd_sendData(unsigned char data)
{
	// for sending the data over 8 pins RS=1 so that the data can be latched
		GPIOA->BSRR=(1<<LCD_RS);
		GPIOB->BRR = 0xF000; // clearing the data pins 
		lcd_putValue(data);

}

void lcd_init()
{
	// for sending the commands EN =0
		GPIOA->BRR =(1<<LCD_EN);
		HAL_Delay(5);
	// It is to be noted that LCD Controller starts up in 8 bit mode and startup instructions are sent in 8 bit mode to send data in 4 bit mode we must perform some initial initilizations before the direct 4 bit mode initialization
		lcd_sendCommand(0x33);
		lcd_sendCommand(0x32);
		lcd_sendCommand(0x28);/* init. in 4bit mode LCD 2 line,5´7 (dot)matrix */
		lcd_sendCommand(0x0C); /* display on, cursor off */
		lcd_sendCommand(0x01); 	/* clear LCD */
		HAL_Delay(2);
		lcd_sendCommand(0x06);/* shift cursor right */
}












void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
