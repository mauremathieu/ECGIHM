/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DOT_MATRIX.h"
#include "DOT_MATRIX_cfg.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE   1500.0f
#define BUFFER_SIZE   4096

#define PI            3.14159265358979323846f
#define SQRT2         1.4142135623730950488f

#define K 			  3  // taille de la fenêtre moy gliss

#define THRESHOLD_BPM			 0.1f   //detection des pics
#define MATRIX_DISPLAY_UNIT1 	 0

#define NUM_PEAKS_FOR_BPM 		50      // nombre de pics à utiliser pour le calcul du BPM

// Paramètres du filtre FIR
#define FIR_ORDER 	       64
#define LP_CUTOFF  		   45.0f       // Hz Freq coupure du filtre passe-bas

// Paramètres du filtre de Kalman
#define KALMAN_R      0.005f  // Variance du bruit de mesure
#define KALMAN_Q      0.0001f // Variance du bruit de processus

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float filtered_buffer[BUFFER_SIZE];
volatile uint16_t buffer_index = 0;

// Coefficients du filtre FIR passe-bas
float fir_coeffs[FIR_ORDER+1] = {0.0f};
float fir_buffer[FIR_ORDER+1] = {0.0f};

// Variables pour le filtre de Kalman
float kalman_x = 0.0f;      // État estimé
float kalman_p = 1.0f;      // Estimation de l'erreur de covariance
float kalman_k = 0.0f;      // Gain de Kalman
float kalman_prev = 0.0f;   // Estimation précédente (pour la dérivée)

float hp_prev = 0.0f;
float raw_prev = 0.0f;

uint16_t adc_buffer[K];
uint8_t adc_count = 0;

volatile uint32_t last_peak_time = 0;
volatile int peak_count = 0;
float bpm = 0.0f;
char bpm_buffer[8];
int cnt_bpm = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void lpf_compute_coeffs(float cutoff, float samplerate);
float apply_kalman_filter(float input);
void process_and_transmit(float avg_val);
float calculate_bpm(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  lpf_compute_coeffs(LP_CUTOFF, SAMPLE_RATE);

  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(&hadc1);


  //oled
  DOT_MATRIX_Init_TMR(&hspi3, &htim2);
  MATRIX_DisplayMessage(MATRIX_DISPLAY_UNIT1, bpm_buffer, sizeof(bpm_buffer));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7)
	{
		//led clignote
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

	//affichage bpm sur oled
	MATRIX_TMR_OVF_ISR(htim);

	cnt_bpm++;
	if (cnt_bpm>40) { //envoie des données avec un compteur pour pas saturer affichage oled
		sprintf(bpm_buffer, "%dbpm", (int)bpm);
		cnt_bpm = 0;
	}
}

//calcul des coefs du passe bas
void lpf_compute_coeffs(float cutoff, float samplerate) {
    // Calcul des coefficients d'un filtre FIR passe-bas avec fenêtre de Hamming
    float fc = cutoff / samplerate;  // Fréquence de coupure normalisée (0 à 0.5)
    int middle = FIR_ORDER / 2;

    // Création du filtre FIR
    for (int i = 0; i <= FIR_ORDER; i++) {
        if (i == middle) {
            // Cas spécial pour éviter la division par zéro
            fir_coeffs[i] = 2.0f * fc;
        } else {
            // Formule du filtre sinus cardinal
            float x = 2.0f * PI * fc * (i - middle);
            fir_coeffs[i] = sin(x) / x;
        }

        // Application de la fenêtre de Hamming
        fir_coeffs[i] *= (0.54f - 0.46f * cos(2.0f * PI * i / FIR_ORDER));
    }

    // Normalisation des coefficients pour un gain unitaire
    float sum = 0.0f;
    for (int i = 0; i <= FIR_ORDER; i++) {
        sum += fir_coeffs[i];
    }

    for (int i = 0; i <= FIR_ORDER; i++) {
        fir_coeffs[i] /= sum;
    }
}


/* filtre FIR */
float apply_lpf_order2(float input) {
    // Décalage du buffer
    for (int i = FIR_ORDER; i > 0; i--) {
        fir_buffer[i] = fir_buffer[i-1];
    }
    fir_buffer[0] = input;

    // Application du filtre FIR
    float output = 0.0f;
    for (int i = 0; i <= FIR_ORDER; i++) {
        output += fir_coeffs[i] * fir_buffer[i];
    }

    return output;
}

/* Filtre de Kalman */
float apply_kalman_filter(float input) {
    // Prédiction
    // x(k|k-1) = x(k-1|k-1) (modèle simple, pas de changement prédit)
    // P(k|k-1) = P(k-1|k-1) + Q
    kalman_p = kalman_p + KALMAN_Q;

    // Mise à jour
    // K(k) = P(k|k-1) / (P(k|k-1) + R)
    kalman_k = kalman_p / (kalman_p + KALMAN_R);

    // x(k|k) = x(k|k-1) + K(k) * (z(k) - x(k|k-1))
    float innovation = input - kalman_x;
    kalman_x = kalman_x + kalman_k * innovation;

    // P(k|k) = (1 - K(k)) * P(k|k-1)
    kalman_p = (1.0f - kalman_k) * kalman_p;

    // Calculer la dérivée pour agir comme un filtre passe-haut
    float kalman_derivative = kalman_x - kalman_prev;
    kalman_prev = kalman_x;

    return kalman_derivative;
}

// ADC callback: collecte des données et calcul de la moyenne après K appels
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t raw_val = HAL_ADC_GetValue(hadc);

    // Stockage dans le buffer temporaire
    adc_buffer[adc_count] = raw_val;
    adc_count++;

    // Après K appels, calculer la moyenne et traiter
    if (adc_count >= K)
    {
        float sum = 0.0f;
        for (int i = 0; i < K; i++)
        {
            // Appliquer la saturation
            float val;
            if (adc_buffer[i] < 800.0f)
            {
                val = (float)adc_buffer[i];
            }
            else
            {
                val = 800.0f;
            }
            sum += val;
        }

        // Calculer la moyenne
        float avg_val = sum / K;

        // Traiter et transmettre
        process_and_transmit(avg_val);

        // Réinitialiser le compteur
        adc_count = 0;
    }
}

// Fonction pour traiter et transmettre la valeur moyenne
void process_and_transmit(float avg_val)
{
    // Filtre passe-bas (LP) ordre 2
    float lp_val = apply_lpf_order2(avg_val);

    // Filtre passe-haut (HP)
    float kalman_val = apply_kalman_filter(lp_val);

    // Stockage circulaire
    filtered_buffer[buffer_index] = kalman_val;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;

    //BPM
    static float squared_val = 0.0f;

    // Tableau pour stocker les horodatages des pics
    static uint32_t peak_times[NUM_PEAKS_FOR_BPM] = {0};
    static int peak_index = 0;

    // Élévation au carré
    squared_val = kalman_val * kalman_val;

    // Détection des pics
    if (squared_val > THRESHOLD_BPM) {
        uint32_t current_time = HAL_GetTick(); // Temps en ms

        // Éviter la détection de plusieurs pics trop rapprochés
        if (current_time - last_peak_time > 200) {
            // Enregistrer l'horodatage du pic actuel
            peak_times[peak_index] = current_time;
            peak_index = (peak_index + 1) % NUM_PEAKS_FOR_BPM; // Stockage circulaire

            // Mettre à jour le dernier temps de pic
            last_peak_time = current_time;
            peak_count++;
        }
    }

    // Calcul du BPM basé sur les derniers pics
    if (peak_count >= 5) {  // min 5 pics pour calculer
        int num_peaks_to_use = (peak_count < NUM_PEAKS_FOR_BPM) ? peak_count : NUM_PEAKS_FOR_BPM;

        int oldest_peak_idx = (peak_index - num_peaks_to_use + NUM_PEAKS_FOR_BPM) % NUM_PEAKS_FOR_BPM;
        uint32_t oldest_time = peak_times[oldest_peak_idx];

        if (oldest_time > 0) {
            // Calcul du BPM basé sur le temps entre le pic le plus ancien et le plus récent
            uint32_t time_span = last_peak_time - oldest_time;
            if (time_span > 0) {
                bpm = ((num_peaks_to_use - 1) * 60000.0f) / (float)time_span;
            }
        }
    }

    // Transmission UART des valeurs pour Serial Plotter
    char tx_buffer[64];
    sprintf(tx_buffer, "%.2f %.2f %.2f %.2f %.2f\n", avg_val, lp_val, kalman_val, squared_val, bpm);
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}

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
