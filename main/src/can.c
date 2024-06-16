#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "app_internal.h"
#include "can.h"
#include "Steering_MeasurementModel.h"
#include "Heading_Measurement.h"
#include "GPS_Measurement.h"
twai_message_t rx_msg;
volatile uint8_t CAN_flag=0;
// uint8_t Data[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
// uint8_t DataLength=16;
uint8_t AccX[4];
uint8_t AccY[4];
uint8_t Lat[4];
uint8_t Long[4];
uint8_t BNOmessage=0;
extern float accx,accy;
extern float EncoderVel;
extern  Steering Steeringexe;
extern Input Inputexe;
extern GPS GPSexe;
extern uint8_t GPSrec;
extern uint8_t Velrec;
extern uint8_t Hearec;
extern float HeadingHandle[4];
extern bool GPSpathOK;
extern uint8_t stee_speed;
uint8_t Velstart=0;
uint8_t PWM;
int cntt=0;
uint8_t SteeringData[8];
uint8_t Headingdriff[4];
uint8_t pid_speed;
uint8_t trans=0;
extern float heading_diff;
/* --------------------------- Tasks and Functions -------------------------- */
static void twai_transmit_task(void *arg){
    for(;;)
    {
     GPSrec=1;
    if(GPSrec==1){
       // printf("OK \r\n");
        twai_message_t message;
        message.identifier = 0x102;
        message.flags=TWAI_MSG_FLAG_NONE;
        message.data_length_code = 8;
        message.data[0]=pid_speed;
        message.data[1]=stee_speed;
        float2Bytes(Headingdriff,heading_diff);
        message.data[2]=Headingdriff[0];
        message.data[3]=Headingdriff[1];
        message.data[4]=Headingdriff[2];
        message.data[5]=Headingdriff[3];
        message.data[6]=0x55;
        message.data[7]=0x55;
if (twai_transmit(&message, portMAX_DELAY) == ESP_OK) {
   trans=1;
} else {
   // printf("Failed to queue message for transmission\n");
}
} 
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}        
static void twai_receive_task(void *arg)
{    
    for (;;)
    {
       // printf("Message OK \r\n");
            if(twai_receive(&rx_msg, portMAX_DELAY)==ESP_OK){
                CAN_flag=1;
               // printf("Message OK\n");
            }else
            {
               // printf("Fail OK\n");
            }
              if(rx_msg.identifier==0x302){
                if(rx_msg.data[0]!=0x55){
                     Hearec=1;
                     Inputexe.Stee =bytes2Float(rx_msg.data);
                     accx=rx_msg.data[4];
                     accy=rx_msg.data[5];
                }
                else
                {
                     Hearec=0;
                }
            }
             if(rx_msg.identifier==0x002){
                if(rx_msg.data[0]!=0x55){
                    Velrec=1;
                    EncoderVel=bytes2Float(rx_msg.data);
                }
                else{
                    Velrec=0;
                }
            }
            if(rx_msg.identifier==0x202){
                if(rx_msg.data[0]!=0x55){
                 GPSrec=1;
                 Velstart=1;
                //printf("GPS RecOk %d\r\n",cntt++);
                Lat[0]=rx_msg.data[0];
                Lat[1]=rx_msg.data[1];
                Lat[2]=rx_msg.data[2];
                Lat[3]=rx_msg.data[3];
                //printf("lat 1 %d || Lat 2 %d\r\n", Lat[0],Lat[1]);
//--------------------------------------------------------------------
                Long[0]=rx_msg.data[4];
                Long[1]=rx_msg.data[5];
                Long[2]=rx_msg.data[6];
                Long[3]=rx_msg.data[7];
                GPSexe.GPSGetPosition[0]=bytes2Float(Lat);
                GPSexe.GPSGetPosition[1]=bytes2Float(Long);
                }
                else
                {
                    GPSrec=0;
                }
                 //printf("Px %f || Py %f\r\n",GPSexe.GPSGetPosition[0],GPSexe.GPSGetPosition[1]);
            }
          
          vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
// static void Steering_task(void *arg) {
//     for(;;)
//     {
//   //  if(Hearec==1){
//         twai_message_t message;
//         message.identifier = 0x402;
//         message.flags=TWAI_MSG_FLAG_NONE;
//         message.data_length_code = 8;
//         message.data[0]=HeadingHandle[0];
//         message.data[1]=HeadingHandle[1];
//         message.data[2]=HeadingHandle[2];
//         message.data[3]=HeadingHandle[3];
//         message.data[4]=0x55;
//         message.data[5]=0x55;
//         message.data[6]=0x55;
//         message.data[7]=0x55;
//        // printf("%u || %u || %u || %u\r\n",message.data[0],message.data[1],message.data[2],message.data[3]);
// if (twai_transmit(&message, portMAX_DELAY) == ESP_OK) {
//    // printf("Message queued for transmission\n");
// } else {
//     //printf("Failed to queue message for transmission\n");
// }
// //} 
//         vTaskDelay(20/portTICK_PERIOD_MS);
//     }
// }   
void can_init()
{
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    // Install TWAI driver
   //Install TWAI driver
   memset(SteeringData,0x55,8);
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }
        // Create tasks, queues, and semaphores
    xTaskCreate(twai_receive_task, "TWAI_rx", 2048 * 5,NULL, RX_TASK_PRIO, NULL);
    xTaskCreate(twai_transmit_task,"TWAI_tx", 2048 * 5,NULL, TX_TASK_PRIO,NULL);
 //   xTaskCreate(Steering_task,"TWAI_tx_stx",  2048 * 5,NULL, 7 ,NULL);
     if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}
