/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum e_transfer_status
{
	E_NOT_READY_FOR_TRANSFER,
	E_READY_FOR_TRANSFER,
}typedef e_transfer_status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// MPU9250 I2C address
#define MPU9250_ADDRESS 0x68 << 1  // Shifted left to match HAL API requirements

// MPU9250 Register Addresses
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_TEMP_OUT_H 0x41

// Configuration registers
#define MPU9250_CONFIG            0x1A
#define MPU9250_GYRO_CONFIG       0x1B
#define MPU9250_ACCEL_CONFIG      0x1C
#define MPU9250_ACCEL_CONFIG_2    0x1D
#define MPU9250_CONFIG            0x1A
#define MPU9250_GYRO_CONFIG       0x1B
#define MPU9250_ACCEL_CONFIG_2    0x1D

// Configuration values
#define DLPF_CFG_184HZ            0x01  // Low pass filter set to 184Hz
#define GYRO_FS_SEL_250DPS        0x00  // Gyro full scale range set to ±250 degrees per second
#define ACCEL_FS_SEL_2G           0x00  // Accelerometer full scale range set to ±2g
#define ACCEL_DLPF_184HZ          0x01  // Accelerometer low pass filter set to 184Hz

// DLPF configuration values
#define DLPF_CFG_5HZ              0x06  // Low pass filter set to 5Hz for gyro
#define ACCEL_DLPF_CFG_5HZ        0x06  // Low pass filter set to 5Hz for accel

#define DLPF_CFG_20HZ             0x04  // Low pass filter set to 20Hz for gyro
#define ACCEL_DLPF_CFG_20HZ       0x04  // Low pass filter set to 20Hz for accel


#define MPU9250_INT_ENABLE         0x38
#define MPU9250_INT_PIN_CFG        0x37
#define MPU9250_INT_DATA_RDY_BIT   0x01

// Define constants
#define RAD_TO_DEG 57.2957795131  // Convert radians to degrees

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
int16_t Temperature;

float Roll = 0, Pitch = 0, Yaw = 0;

float Roll_filter = 0, Pitch_filter = 0, Yaw_filter = 0;

uint32_t prev_time = 0, curr_time = 0;
float dt = 0;
float g_elapsedTime = 0;


// PID
float roll_desire = 0, pitch_desire = 0, yaw_desire = 0;
float pid_roll = 0, pid_roll_p = 0, pid_roll_i = 0, pid_roll_d = 0;
float pid_pitch = 0, pid_pitch_p = 0, pid_pitch_i = 0, pid_pitch_d = 0;
float pid_yaw = 0, pid_yaw_p = 0, pid_yaw_i = 0, pid_yaw_d = 0;
int32_t pid_roll_previous_error = 0, pid_roll_error = 0;
int32_t pid_pitch_previous_error = 0, pid_pitch_error = 0;
int32_t pid_yaw_previous_error = 0, pid_yaw_error = 0;


float kp_roll=0.01;//3.55
float ki_roll=0.0002;//0.003
float kd_roll=0.01;//2.05

float kp_pitch=0.01;//3.55
float ki_pitch=0.0002;//0.003
float kd_pitch=0.01;//2.05

float kp_yaw=0.01;//3.55
float ki_yaw=0.0002;//0.003
float kd_yaw=0.01;//2.05

uint32_t throtle = 1050;

uint8_t start_calculate_pid = 0;

uint32_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

uint8_t receive_data[5] = {0};
uint8_t index_count_receive_data = 0;

char buffer[64];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void calculate_pid()
{
	if(throtle > 1150)
	{
		pid_roll_error  = Roll_filter - roll_desire;
		pid_pitch_error = Pitch_filter - pitch_desire;
		pid_yaw_error   = Yaw_filter - yaw_desire;

//		if((-0.25 < pid_roll_error) && (pid_roll_error < 0.25))
//		{
//			pid_roll_error = 0;
//		}
//
//		if((-0.25 < pid_pitch_error) && (pid_pitch_error < 0.25))
//		{
//			pid_pitch_error = 0;
//		}
//
//		if((-0.25 < pid_yaw_error) && (pid_yaw_error < 0.25))
//		{
//			pid_yaw_error = 0;
//		}

//		if((-0.5 <= pid_roll_error) && (pid_roll_error <= -0.5))
//		{
//			kp_roll = 0.07;
//			ki_roll = 0.001;
//			kd_roll = 0.05;
//		}
//		else if((-1 <= pid_roll_error) && (pid_roll_error <= 1))
//		{
//			kp_roll = 0.1;
//			ki_roll = 0.002;
//			kd_roll = 0.05;
//		}
//		else if((-2 <= pid_roll_error) && (pid_roll_error <= 2))
//		{
//			kp_roll = 0.3;
//			ki_roll = 0.005;
//			kd_roll = 0.15;
//		}
//		else if((-3 <= pid_roll_error) && (pid_roll_error <= 3))
//		{
//			kp_roll = 0.35;
//			ki_roll = 0.01;
//			kd_roll = 0.15;
//		}
//		else if((-4 <= pid_roll_error) && (pid_roll_error <= 4))
//		{
//			kp_roll = 0.4;
//			ki_roll = 0.01;
//			kd_roll = 0.2;
//		}
//		else if((-8 <= pid_roll_error) && (pid_roll_error <= 8))
//		{
//			kp_roll = 0.4;
//			ki_roll = 0.015;
//			kd_roll = 0.2;
//		}
//		else if((-10 <= pid_roll_error) && (pid_roll_error <= 10))
//		{
//			kp_roll = 0.5;
//			ki_roll = 0.02;
//			kd_roll = 0.25;
//		}
//		else if((-20 <= pid_roll_error) && (pid_roll_error <= 20))
//		{
//			kp_roll = 0.55;
//			ki_roll = 0.02;
//			kd_roll = 0.25;
//		}
//		else
//		{
//			kp_roll = 0.55;
//			ki_roll = 0.025;
//			kd_roll = 0.25;
//		}
//
//		if((-0.5 <= pid_pitch_error) && (pid_pitch_error <= -0.5))
//		{
//			kp_pitch = 0.07;
//			ki_pitch = 0.001;
//			kd_pitch = 0.05;
//		}
		 if((-1 <= pid_pitch_error) && (pid_pitch_error <= 1))
		{
			kp_pitch = 0.1;
			ki_pitch = 0.002;
			kd_pitch = 0.05;
		}
		else if((-2 <= pid_pitch_error) && (pid_pitch_error <= 2))
		{
			kp_pitch = 0.3;
			ki_pitch = 0.005;
			kd_pitch = 0.15;
		}
		else if((-3 <= pid_pitch_error) && (pid_pitch_error <= 3))
		{
			kp_pitch = 0.35;
			ki_pitch = 0.07;
			kd_pitch = 0.15;
		}
		else if((-4 <= pid_pitch_error) && (pid_pitch_error <= 4))
		{
			kp_pitch = 0.4;
			ki_pitch = 0.01;
			kd_pitch = 0.2;
		}
		else if((-8 <= pid_pitch_error) && (pid_pitch_error <= 8))
		{
			kp_pitch = 0.4;
			ki_pitch = 0.015;
			kd_roll = 0.2;
		}
		else if((-10 <= pid_pitch_error) && (pid_pitch_error <= 10))
		{
			kp_pitch = 0.5;
			ki_pitch = 0.02;
			kd_pitch = 0.25;
		}
		else if((-20 <= pid_pitch_error) && (pid_pitch_error <= 20))
		{
			kp_pitch = 0.55;
			ki_pitch = 0.025;
			kd_pitch = 0.25;
		}
		else
		{
			kp_pitch = 0.55;
			ki_pitch = 0.03;
			kd_pitch = 0.25;
		}


		if((-1 <= pid_yaw_error) && (pid_yaw_error <= 1))
		{
			kp_yaw = 0.1;
			ki_yaw = 0.003;
			kd_yaw = 0.05;
		}
		else if((-2 <= pid_yaw_error) && (pid_yaw_error <= 2))
		{
			kp_yaw = 0.3;
			ki_yaw = 0.005;
			kd_yaw = 0.15;
		}
		else if((-3 <= pid_yaw_error) && (pid_yaw_error <= 3))
		{
			kp_yaw = 0.35;
			ki_yaw = 0.005;
			kd_yaw = 0.15;
		}
		else if((-4 <= pid_yaw_error) && (pid_yaw_error <= 4))
		{
			kp_yaw = 0.4;
			ki_yaw = 0.005;
			kd_yaw = 0.2;
		}
		else if((-8 <= pid_yaw_error) && (pid_yaw_error <= 8))
		{
			kp_yaw = 0.4;
			ki_yaw = 0.005;
			kd_yaw = 0.2;
		}
		else if((-10 <= pid_yaw_error) && (pid_yaw_error <= 10))
		{
			kp_yaw = 0.5;
			ki_yaw = 0.005;
			kd_yaw = 0.25;
		}
		else
		{
			kp_yaw = 0.55;
			ki_yaw = 0.005;
			kd_yaw = 0.25;
		}

		pid_roll_p  = kp_roll * pid_roll_error;
		pid_pitch_p = kp_pitch * pid_pitch_error;
		pid_yaw_p   = kp_yaw * pid_yaw_error;


	   float old_pid_roll_i = pid_roll_i;
	   pid_roll_i = pid_roll_i+(ki_roll*pid_roll_error);
	   if(pid_roll_i > 0)
	   {
		   if((throtle + pid_roll_i) > 2000)
		   {
			   pid_roll_i = old_pid_roll_i;
		   }
	   }
	   else
	   {
		   if((throtle - pid_roll_i) < 1150)
		   {
			   pid_roll_i = old_pid_roll_i;
		   }
	   }

	   float old_pid_pitch_i = pid_pitch_i;
	   pid_pitch_i = pid_pitch_i+(ki_pitch*pid_pitch_error);

	   if(pid_pitch_i > 0)
	   {
		   if((throtle + pid_pitch_i) > 2000)
		   {
				pid_pitch_i = old_pid_pitch_i;
		   }
	   }
	   else
	   {
		   if((throtle - pid_pitch_i) < 1150)
		   {
				pid_pitch_i = old_pid_pitch_i;
		   }
	   }

	   pid_yaw_i = pid_yaw_i+(ki_yaw*pid_yaw_error);


		pid_roll_d = ki_roll*((pid_roll_error - pid_roll_previous_error)/dt);
		pid_pitch_d = ki_pitch*((pid_pitch_error - pid_pitch_previous_error)/dt);
		pid_yaw_d = ki_yaw*((pid_yaw_error - pid_yaw_previous_error)/dt);


		pid_roll  = pid_roll_p + pid_roll_i + pid_roll_d;
		pid_pitch = pid_pitch_p + pid_pitch_i + pid_pitch_d;
		pid_yaw   = pid_yaw_p + pid_yaw_i + pid_yaw_d;

		pid_roll_previous_error = pid_roll_error;
		pid_pitch_previous_error = pid_pitch_error;
		pid_yaw_previous_error = pid_yaw_error;
	}
	else
	{
		pid_roll_error  = 0;
		pid_pitch_error = 0;
		pid_yaw_error   = 0;
		pid_roll_previous_error = 0;
		pid_pitch_previous_error = 0;
		pid_yaw_previous_error = 0;

		pid_roll_p = 0;
		pid_roll_i = 0;
		pid_roll_d = 0;

		pid_pitch_p = 0;
		pid_pitch_i = 0;
		pid_pitch_d = 0;

		pid_yaw_p = 0;
		pid_yaw_i = 0;
		pid_yaw_d = 0;
	}
}


void SetBrushMotor()
{
  if(throtle > 1050)
  {
	  if(m1 <= 2000)
	  {
		m1 = throtle + pid_roll - pid_pitch;// + pid_yaw;
		if(m1 < 1050)
		{
			m1 = 1050;
		}
	  }
	  else
	  {
		  m1 = 2000;
	  }

	  if(m2 <= 2000)
	  {
		m2 = throtle + pid_roll + pid_pitch;// - pid_yaw;
		if(m2 < 1050)
		{
			m2 = 1050;
		}
	  }
	  else
	  {
		  m2 = 2000;
	  }

	  if(m3 <= 2000)
	  {
		m3 = throtle - pid_roll + pid_pitch;// - pid_yaw;;
		if(m3 < 1050)
		{
			m3 = 1050;
		}
	  }
	  else
	  {
		  m3 = 2000;
	  }

	  if(m4 <= 2000)
	  {
		m4 = throtle - pid_roll - pid_pitch;// + pid_yaw;
		if(m4 < 1050)
		{
			m4 = 1050;
		}
	  }
	  else
	  {
		  m4 = 2000;
	  }

  }

  else
  {
	  m1 = 1050;
	  m2 = 1050;
	  m3 = 1050;
	  m4 = 1050;

  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m4);
}

void MPU9250_Init(void) {
    uint8_t data;

    // Wake up MPU9250 and set clock source
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 1, &data, 1, 1000);

    // Configure DLPF for gyro to 5Hz
    data = DLPF_CFG_5HZ;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_CONFIG, 1, &data, 1, 1000);

    // Set gyro full scale to ±250dps for lowest noise
    data = GYRO_FS_SEL_250DPS;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1, &data, 1, 1000);

    // Set accel full scale to ±2g for lowest noise
    data = ACCEL_FS_SEL_2G;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1, &data, 1, 1000);

    // Configure DLPF for accel to 5Hz
    data = ACCEL_DLPF_CFG_5HZ;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG_2, 1, &data, 1, 1000);
}


void MPU9250_Read_Accel(int16_t *Accel_X, int16_t *Accel_Y, int16_t *Accel_Z) {
    uint8_t Rec_Data[6];

    // Read 6 bytes of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

    // Convert the received data into 16-bit values
    *Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    *Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    *Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}


void MPU9250_Read_Gyro(int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z) {
    uint8_t Rec_Data[6];

    // Read 6 bytes of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 1, Rec_Data, 6, 1000);

    // Convert the received data into 16-bit values
    *Gyro_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    *Gyro_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    *Gyro_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void MPU9250_Read_Temp(int16_t *Temperature) {
    uint8_t Rec_Data[2];

    // Read 2 bytes of data starting from TEMP_OUT_H register
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU9250_TEMP_OUT_H, 1, Rec_Data, 2, 1000);

    // Convert the received data into 16-bit value
    *Temperature = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
}

#define MPU_6050_FIFO 100
uint32_t filter_index = 0;
float Roll_filter_array[MPU_6050_FIFO] = {0}, Pitch_filter_array[MPU_6050_FIFO] = {0}, Yaw_filter_array[MPU_6050_FIFO] = {0};


void MPU9250_EnableInterrupt(void) {
    uint8_t data;

    // Configure the INT pin (active high, push-pull, latched until read)
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 1, &data, 1, 1000);

    // Enable the data ready interrupt
    data = MPU9250_INT_DATA_RDY_BIT;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU9250_INT_ENABLE, 1, &data, 1, 1000);
}

float finding_value(float array[], int length)
{
   float min = array[0];
   float max = array[0];

   uint32_t count_min = 0, count_max = 0;

   for(uint8_t i = 1; i < length; i++)
   {
	   if (fabs(min) > fabs(array[i]))
	   {
		   min = array[i];
	   }

	   if (fabs(max) < fabs(array[i]))
	   {
		   max = array[i];
	   }
   }

   if((fabs(max) - fabs(min)) < 3)
   {
       return min;
   }
   else
   {
	   for(uint8_t i = 1; i < length; i++)
	   {
		   if((fabs(max) - fabs(array[i])) < 3)
		   {
		       count_max++;
		   }

		   if((fabs(array[i]) - fabs(min)) < 3)
		   {
			   count_min++;
		   }
	   }

	   if(count_max > count_min)
	   {
            return max;
	   }
	   else if (count_max < count_min)
	   {
            return min;
	   }
	   else
	   {
		   return (min+max)/2;
	   }
   }

}

void Calculate_Roll_Pitch_Yaw_CompFilter(int16_t Accel_X, int16_t Accel_Y, int16_t Accel_Z,
                                         int16_t Gyro_X, int16_t Gyro_Y, int16_t Gyro_Z,
                                         float *roll, float *pitch, float *yaw, float dt)
{

    // Convert accelerometer readings to g units
    float ax = Accel_X / 16384.0;  // Assuming FS_SEL = ±2g
    float ay = Accel_Y / 16384.0;
    float az = Accel_Z / 16384.0;

    // Convert gyroscope readings to degrees per second
    float gx = Gyro_X / 131.0;  // Assuming FS_SEL = ±250 degrees/s
    float gy = Gyro_Y / 131.0;
    float gz = Gyro_Z / 131.0;

    // Calculate roll and pitch using accelerometer data
    float accel_roll  = atan2(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Integrate gyroscope data to calculate roll, pitch, and yaw
    *roll = 0.98 * (*roll + gx * dt) + 0.02 * accel_roll;
    *pitch = 0.98 * (*pitch + gy * dt) + 0.02 * accel_pitch;
    *yaw += gz * dt;

    Roll_filter_array[filter_index%MPU_6050_FIFO] = *roll;
    Pitch_filter_array[filter_index%MPU_6050_FIFO] = *pitch;
    Yaw_filter_array[filter_index%MPU_6050_FIFO] = *yaw;
    filter_index++;
}
void Validate_Roll_Pitch_Yaw()
{
	Roll_filter = finding_value(Roll_filter_array, MPU_6050_FIFO);
	Pitch_filter = finding_value(Pitch_filter_array, MPU_6050_FIFO);
	Yaw_filter = finding_value(Yaw_filter_array, MPU_6050_FIFO);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);;
    if (GPIO_Pin == MPU9250_INT_Pin) {

        dt = (curr_time - prev_time) / 1000.0;  // Calculate delta time in seconds
        prev_time = curr_time;

        // Read accelerometer and gyroscope data
        MPU9250_Read_Accel(&Accel_X, &Accel_Y, &Accel_Z);
        MPU9250_Read_Gyro(&Gyro_X, &Gyro_Y, &Gyro_Z);

        // Apply the complementary filter
        Calculate_Roll_Pitch_Yaw_CompFilter(Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z, &Roll, &Pitch, &Yaw, dt);

        start_calculate_pid = 1;
    }
}

e_transfer_status ready_to_convert = E_NOT_READY_FOR_TRANSFER;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	index_count_receive_data+=5;
    ready_to_convert = E_READY_FOR_TRANSFER;
}

void converdata()
{
    if(ready_to_convert == E_READY_FOR_TRANSFER)
    {

    	ready_to_convert = E_NOT_READY_FOR_TRANSFER;

    	if(receive_data[0] == 'u')
    	{
    		throtle = (receive_data[1] - 48)*1000 + (receive_data[2] - 48)*100 + (receive_data[3] - 48) * 10 + (receive_data[4] - 48);
    	}
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&receive_data, 5);
    }
}

void UART_Print(const char* str)
{
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, strlen(str));
}


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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  prev_time = HAL_GetTick();
  MPU9250_Init();
  MPU9250_EnableInterrupt();

  HAL_UART_Receive_IT(&huart1, &receive_data[index_count_receive_data], 5);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *) "ready\n", 6);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1050);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1050);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      curr_time = HAL_GetTick();
      converdata();
      if(start_calculate_pid == 1)
      {
          // Use the roll, pitch, and yaw filter values as needed
          Validate_Roll_Pitch_Yaw();
    	  calculate_pid();
    	  SetBrushMotor();
    	  start_calculate_pid = 0;

		  g_elapsedTime += dt;
		  if(g_elapsedTime > 0.05f)
		  {
			  g_elapsedTime = 0;
			  snprintf(buffer, sizeof(buffer), "Roll: %d Pitch: %d Yaw: %d\n", (int) Roll_filter, (int) Pitch_filter, (int)Yaw_filter);
			  UART_Print(buffer);
		  }
    	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);;
      }

	  //HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20160;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : MPU9250_INT_Pin */
  GPIO_InitStruct.Pin = MPU9250_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU9250_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
