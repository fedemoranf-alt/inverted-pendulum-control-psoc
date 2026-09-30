/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include <common.h>


//Definiciones del sistema de adquisicion>
#define K_ADC 9.37 //ganancia del circuito de acondicionamiento del ADC
//#define K_ADC 8.5
#define ADC_offset 2.4 //offset del circuito de acondicionamiento del ADC
#define R 1000.0 //ajuste de escala del DAC. Ej: 1000 ---> 1 volt
#define K_DAC 2.47 //Ganancia del circuito de acondicionamiento del DAC

uint8 errorStatus = 0u;

int X;
int T;

float X_met;
float T_rad;

float ref = 0.0;
float e = 0.0;
float e_1 = 0.0;
float e_2 = 0.0;
float e_3 = 0.0;
float u = 0.0;
float u_1 = 0.0;
float u_2 = 0.0;
float u_3 = 0.0;

int flag = '3';

/* Add an explicit reference to the floating point printf library to allow
the usage of floating point conversion specifier */
#if defined (__GNUC__)
    asm (".global _printf_float");
#endif

//Rutina de servicios de interrupcion
CY_ISR_PROTO(Adquisicion);
CY_ISR(RxIsr);


void DAC_Write(float value){
    float out;
    
    /*Tope para los valores fuera de rango*/
    value = value > 10.0 ? 10.0 : value ;
    value = value < -10.0 ? -10.0 : value;
    
    //tener en cuenta que los dacs no pueden escribir ambos al mismo tiempo, uno de ellos tiene que estar en tierra
    if(value >=0.0){
     DAC_N_SetValue(0);
     out = (( (value)/(1 + K_DAC) )*255.0)/4.08;
     DAC_P_SetValue( (uint8) out );
    
    } else {
     DAC_P_SetValue(0);
     out = (( -(value)/( K_DAC) )*255.0)/4.08;
     DAC_N_SetValue( (uint8) out );
    }
}

void My_Start(){
    //inicializacion de los componentes de salida del sistema:
    DAC_P_Start();
    DAC_N_Start();
    DAC_P_SetValue(0);
    DAC_N_SetValue(0);
    Opamp_1_Start();
    Timer_1_Start();
    UART_Start(); 
    
   
    //inicializacion de los componentes de entrada del sistema:
    QuadDec_X_Start();
    QuadDec_X_Enable();
    QuadDec_T_Start();
    QuadDec_T_Enable();
    timer_clock_Start();
  
    //CyDelay(50);

    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    //rutina de servicio de interrupcion:
    isr_1_StartEx(Adquisicion);
    isr_rx_StartEx(RxIsr);
    isr_rx_Enable();
   // isr_1_Enable();

}


int main(void)
{
    My_Start();

    for(;;)
    {   
        if (flag == '0'){
            
        }
        
        if(flag == '1'){ 
            //esto se hace porque primero necesitamos apretar la planta y luego realizar la primera medicion, la y en el estimador actual es y(k+1), es decir nos 
            //adelantamos un paso a la medida para tener y(k+1) al medir con el adc luego
           /* u = N_bar*ref - (K[0]*xea1[0] + K[1]*xea2[0]);
            DAC_Write(u); 
            */
            QuadDec_T_SetCounter(0);
            QuadDec_X_SetCounter(0);
            u = 0.0;
            u_1 = 0.0;
            u_2 = 0.0;
            u_3 = 0.0;
            e_1 = 0.0;
            e_2 = 0.0;
            e_3 = 0.0;
            isr_1_Enable();
            flag = '0';
        }
        
        if(flag =='2'){
            flag = '0';
            
             X = QuadDec_X_GetCounter();
            X_met=(2*3.14*(X/1040.0)*0.0181); 
            //X_met = (X*0.6*3.14/180.0)*0.0181;
            //X_vel = (X_met - X_met_1)/0.01;
            T = QuadDec_T_GetCounter();
            T_rad = -(T*0.15*3.14/180.0);
            //T_rad = (2.0*3.14*(T/2400.0))-3.14*sgn(T);
           // T_vel = (T_rad - T_rad_1)/0.01;
            //printf("T%f\r\n", T_rad);
            //printf("X%d\r\n", X);
            
            e = ref - T_rad; //se calcula el error
            
            //se calcula el esfuerzo de control con la ecuacion en diferencia del compensador            
            
            //PARA 1MS RL
            u = 1631*e - 2953*e_1 + 1346*e_2 + 1.6*u_1 - 0.65*u_2 ; //ANDA
            
            //BODE 1MS
            // u = 1013*e - 2003*e_1 + 990.3*e_2 + 1.889*u_1 - 0.8997*u_2 ;
             //u = 1609*e - 3174*e_1 + 1566*e_2 + 1.86*u_1 - 0.88*u_2 ;
           
            
            printf("%f,%f\r\n", u, T_rad);
            //para el anti winup
            if(u>=400.0 | u <= -400.0){
                u = 0.0;
                u_1 = 0.0;
                u_2 = 0.0;
                u_3 = 0.0;
                e_1 = 0.0;
                e_2 = 0.0;
                e_3 = 0.0;;
            }
            DAC_Write(u);
                    
            u_1 = u;
            u_2 = u_1;
            u_3 = u_2;
            e_1 = e;
            e_2 = e_1;
            e_3 = e_2;
                     
        }
        if (flag == '3'){
            isr_1_Disable();
             DAC_Write(0);
        }
        
        /*if(flag == '4'){ //tomar referencia, se toma cuando esta quieto por la gravedad para que sea -180 asi al subir esta en cero
            QuadDec_T_SetCounter(1200);
            isr_1_Disable();
            DAC_Write(0);
            flag = '0';
        }*/
    }
}

CY_ISR_PROTO(Adquisicion){
   
    flag = '2';
    isr_1_ClearPending();
}



CY_ISR(RxIsr)
{
    
    uint8 rxStatus;
    char rxData;  
    
    do
    {
        //Read receiver status register 
        rxStatus = UART_RXSTATUS_REG;

        if((rxStatus & (UART_RX_STS_BREAK      | UART_RX_STS_PAR_ERROR |
                        UART_RX_STS_STOP_ERROR | UART_RX_STS_OVERRUN)) != 0u)
        {
            // ERROR handling. 
            errorStatus |= rxStatus & ( UART_RX_STS_BREAK      | UART_RX_STS_PAR_ERROR | 
                                        UART_RX_STS_STOP_ERROR | UART_RX_STS_OVERRUN);
        }
        
        if((rxStatus & UART_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            // Read data from the RX data register 
            rxData = UART_RXDATA_REG;
            
            if(rxData == 'I'){
             flag = '1';   
            }
            if(rxData == 'P'){
             flag = '3';   
            }
          /*  if(rxData == 'R'){
             flag = '4';   
            }*/
            
            if(errorStatus == 0u)
            {
                //Send data backward 
                UART_TXDATA_REG = rxData;
            }
            
        }
    }while((rxStatus & UART_RX_STS_FIFO_NOTEMPTY) != 0u);
    //isr_rx_ClearPending(); 
}



/* [] END OF FILE */