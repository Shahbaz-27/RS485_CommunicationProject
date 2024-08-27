
#include "main.h"

void GPIO_Initialization();
void USART_Initialization();
void LED_Control(uint8_t number);
uint8_t receiveData();


void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
	/*MX Initializations*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	
	
	
	/*Bitwise Initializations*/
	GPIO_Initialization();
	USART_Initialization();
	
  while (1)
  {
  uint8_t number = receiveData();
	LED_Control(number);
	}

}

void LED_Control(uint8_t number)
{
	if(number == 1)
	{
		GPIOA->BSRR = (1<<0); 
	}
	else if(number == 2)
	{
			GPIOA->BSRR = (1<<1); 

	}
	
	else if(number == 3)
	{
			GPIOA->BSRR = (1<<2); 

	}
	
	else if(number == 4)
	{
			GPIOA->BSRR = (1<<3); 
	}

	
	else if(number == 5)
	{
			GPIOA->BSRR = (1<<7); 

	}
	
	else if(number == 6)
	{
			GPIOB->BSRR = (1<<0); 

	}
	
	else if(number == 7)
	{
			GPIOB->BSRR = (1<<1); 

	}
	
	else if(number == 8)
	{
			GPIOB->BSRR = (1<<10); 

	}
	
	else if(number == 9)
	{
			GPIOB->BSRR = (1<<11); 

	}
	
	else 
	{
		GPIOA->BRR |=0x00FF; // As the user presses 0 all the LEDs goes OFF
		GPIOB->BRR |=0x0FFF; // As the user presses 0 all the LEDs goes OFF
	}	
}

uint8_t receiveData()
{
  HAL_Delay(200);	
	while((USART1->SR&(1<<5)) == 0); // wait until complete data is received
	return USART1->DR;
}


void USART_Initialization()
{
	RCC->APB2ENR |= (1<<14);
	
	GPIOA->CRH    = 0x444448B4; /* UART Pins */
	GPIOA->ODR   |= (1<<10); /*Pull-up RX of USART1*/	
	USART1->CR1   = 0x200C;//
	USART1->BRR   = 6666; /*for B.rate of 9600 @64Mhz*/
}
void GPIO_Initialization()
{
	RCC->APB2ENR |= (0x0C);
	GPIOA->CRL = 0x33333333;
	GPIOB->CRL = 0x44444433;
	GPIOB->CRH = 0x44443344;
 }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
