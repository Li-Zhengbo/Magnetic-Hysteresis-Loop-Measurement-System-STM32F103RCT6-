/* USER CODE BEGIN Includes */
#include "main.h"
#include <math.h>                      //提供数学函数(sqrt/sin/......)
#include <stdio.h>                     //提供snprintf安全格式字符序列函数(避免内存越界)
#include <string.h>                    //字符序列处理支持
/* USER CODE END Includes */


/* USER CODE BEGIN PV */
//系统使用参数定义
#define ADC_BUF_LEN 256                //DMA缓存数据尺寸(256)
#define ADC_RESOLUTION 4096.0f         //ADC Resolution(12 bits)
#define VREF 3.3f                      //ADC参考电压(V)
#define FREQ 1000.0f                   //激励频率(Hz)
#define N2 100.0f                      //N2绕组
#define AREA 3.14e-4f                  //N2绕组磁芯面积(m²)
#define MU0 (4e-7f * 3.1415926f)       //空气μ₀(4π×10⁻⁷)
#define N1 50.0f                       //N1绕组
#define L_MAG 0.05f                    //磁路尺寸(m)
#define R_SHUNT 0.1f                   //励磁电流分流电阻(Ω)
#define A_PRIME 4.0e-4f                //试样材料修正面积(B)

//DMA 缓冲数据&状态标志
uint16_t adc1_buf[ADC_BUF_LEN];        //ADC1(采集N2绕组感应电压)
uint16_t adc2_buf[ADC_BUF_LEN];        //ADC2(采集N1绕组串联分流精密电阻电压)
volatile uint8_t dma1_done = 0;        //ADC1 DMA 完成标志
volatile uint8_t dma2_done = 0;        //ADC2 DMA 完成标志
/* USER CODE END PV */


/* USER CODE BEGIN 1 */
//系统初始代码
HAL_Init();                            //初始HAL
SystemClock_Config();                  //配置系统时钟
MX_GPIO_Init();                        //初始GPIO
MX_DMA_Init();                         //初始DMA
MX_ADC1_Init();                        //初始ADC1
MX_ADC2_Init();                        //初始ADC2
MX_TIM2_Init();                        //初始TIM
MX_USART1_UART_Init();                 //初始USART1
MX_USART2_UART_Init();                 //初始USART2

//启动外设
HAL_TIM_Base_Start(&htim2);                                     //启动TIM(定时触发ADC)
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_LEN);    //启动 ADC1&DMA1
HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);    //启动 ADC2&DMA2
/* USER CODE END 1 */


/* USER CODE BEGIN WHILE */
char uart1_buf[128];  //USART1数据缓存
char uart2_buf[128];  //USART2数据缓存

while (1)
{
    //判断 2 DMA 全部完成搬运
    if (dma1_done && dma2_done)
    {
        dma1_done = 0;
        dma2_done = 0;

        float sum_U2 = 0, sum_I = 0;                                   //∑
        for (int i = 0; i < ADC_BUF_LEN; i++) {
            float u2 = ((float)adc1_buf[i]) * VREF / ADC_RESOLUTION;   //ADC1转换电压(感应绕组)
            float vi = ((float)adc2_buf[i]) * VREF / ADC_RESOLUTION;   //ADC2转换电压(励磁电流)
            sum_U2 += u2;
            sum_I  += vi;
        }

        float U2_avg = sum_U2 / ADC_BUF_LEN;                           //平均感应电压
        float I_avg  = sum_I / ADC_BUF_LEN;                            //平均励磁采样电压

        float I = I_avg / R_SHUNT;                                     //计算励磁电流(I = V/R)
        float H = N1 * I / L_MAG;                                      //计算磁场强度(H = N1*I/l)
        float B_raw = U2_avg / (4.0f * FREQ * AREA * N2);              //计算原始B(积分换算)
        float B_corr = B_raw - MU0 * H * ((A_PRIME - AREA) / AREA);    //修正B(考虑面积偏差)

        //USART1打印B信息
        int len1 = snprintf(uart1_buf, sizeof(uart1_buf),
            "[B] raw=%.6f T, Bcorr=%.6f T\r\n",
            B_raw, B_corr);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart1_buf, len1, HAL_MAX_DELAY);

        //USART2打印H信息
        int len2 = snprintf(uart2_buf, sizeof(uart2_buf),
            "[H] = %.2f A/m\r\n", H);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart2_buf, len2, HAL_MAX_DELAY);
    }
    HAL_Delay(10);                                                     //循环延时(限制限速保证数据处理完成)
/* USER CODE END WHILE */


/* USER CODE BEGIN 4 */
//ADC&DMA 传输完成回调函数(ADC&DMA 完成缓冲采样自动使用回调函数)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) dma1_done = 1;  // ADC1 DMA 完成
    if (hadc->Instance == ADC2) dma2_done = 1;  // ADC2 DMA 完成
}
/* USER CODE END 4 */
