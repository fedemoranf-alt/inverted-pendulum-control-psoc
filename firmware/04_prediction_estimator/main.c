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
float T = 0.02;

float X_met;
float X_met_1; //valor anterior de x_met
float X_vel; //derivada de x_met, velocidad del carrito
float T_rad;
float T_rad_1; //valor anterior de T_rad
float T_vel;

//Definicion del sistema
 float A [4][4]= { {1.0000,    0.0303,    0.0029,    0.0000},
            {0,    0.5548,    0.1333,    0.0029},
            {0,   -0.0353,    1.0389,    0.0405},
            {0,   -1.6199,    1.9096,    1.0389 }};

float B[4][1] = { {0.0007},
                {0.0338},
                {0.0027},
                {0.1230}};

float C[2][4] = {{1,     0,     0,     0},
                {0,     0,     1,     0}};

//40MS FUNCIONA, POLOS del carro en 0.4 + 0.4 con modelado del motor, CON PELOTA, EL MEJOR
float K[] ={ -334.3859, -141.8694,  306.5953,   55.5746};

//estimador de prediccion
float xep1[] ={0.0, 0.0, 0.0}; 
float xep2[] ={0.0, 0.0, 0.0}; 
float xep3[] ={0.0, 0.0, 0.0}; 
float xep4[] ={0.0, 0.0, 0.0}; 
float Kep [4][2] = {   {1.0645,    0.0001},
    {40.3804  ,  0.0111},
    {-0.4019 ,   1.2031},
    {-34.5210  , 52.4734}};


float ref = 0.0;
float e = 0.0;
float e_1 = 0.0;
float e_2 = 0.0;
float u = 0.0;
float u_1 = 0.0;
float u_2 = 0.0;

int flag = '1';

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
            e_1 = 0.0;
            e_2 = 0.0;
            X_met = 0.0;
            X_met_1 = 0.0;
            X_vel = 0.0;
            T_rad = 0.0;
            T_rad_1 = 0.0;
            T_vel = 0.0;
            
            xep1[0] = 0;
            xep2[0] = 0; 
            xep3[0] = 0;
            xep4[0] = 0; 
            
            isr_1_Enable();
            flag = '0';
        }
        
        if(flag =='2'){
            flag = '0';
            
            X = QuadDec_X_GetCounter();
            X_met = (X*0.6*3.14/180.0)*0.0181;
            T = QuadDec_T_GetCounter();
            T_rad = T*0.15*3.14/180.0;
            
            //printf("T%f\r\n", T_rad);
            //printf("X%f\r\n", X_met);       
            
            //se calcula el esfuerzo de control con la ecuacion en diferencia del compensador                        
           // u = ref - (K[0]*xep1[0] + K[1]*xep2[0] + K[2]*xep3[0] + K[3]*xep4[0]);
            u = ref - (K[0]*xep1[0] + K[2]*xep3[0]);
            
            //se calcula el estimador de estados de prediccion
            xep1[1] = (A[0][0]*xep1[0] + A[0][1]*xep2[0] + A[0][2]*xep3[0] + A[0][3]*xep4[0]) + B[0][0]*u + 
                        Kep[0][0]*(T_rad - (C[0][0]*xep1[0] + C[0][1]*xep2[0] + C[0][2]*xep3[0] + C[0][3]*xep4[0])) + 
                        Kep[0][1]*(X_met - (C[1][0]*xep1[0] + C[1][1]*xep2[0] + C[1][2]*xep3[0] + C[1][3]*xep4[0]));
            
            xep2[1] = (A[1][0]*xep1[0] + A[1][1]*xep2[0] + A[1][2]*xep3[0] + A[1][3]*xep4[0]) + B[1][0]*u + 
                        Kep[1][0]*(T_rad - (C[0][0]*xep1[0] + C[0][1]*xep2[0] + C[0][2]*xep3[0] + C[0][3]*xep4[0])) + 
                        Kep[1][1]*(X_met - (C[1][0]*xep1[0] + C[1][1]*xep2[0] + C[1][2]*xep3[0] + C[1][3]*xep4[0]));
                        
            xep3[1] = (A[2][0]*xep1[0] + A[2][1]*xep2[0] + A[2][2]*xep3[0] + A[2][3]*xep4[0]) + B[2][0]*u + 
                        Kep[2][0]*(T_rad - (C[0][0]*xep1[0] + C[0][1]*xep2[0] + C[0][2]*xep3[0] + C[0][3]*xep4[0])) + 
                        Kep[2][1]*(X_met - (C[1][0]*xep1[0] + C[1][1]*xep2[0] + C[1][2]*xep3[0] + C[1][3]*xep4[0]));

            xep4[1] = (A[3][0]*xep1[0] + A[3][1]*xep2[0] + A[3][2]*xep3[0] + A[3][3]*xep4[0]) + B[3][0]*u + 
                        Kep[3][0]*(T_rad - (C[0][0]*xep1[0] + C[0][1]*xep2[0] + C[0][2]*xep3[0] + C[0][3]*xep4[0])) + 
                        Kep[3][1]*(X_met - (C[1][0]*xep1[0] + C[1][1]*xep2[0] + C[1][2]*xep3[0] + C[1][3]*xep4[0]));
            
            xep1[0] = xep1[1];
            xep2[0] = xep2[1]; 
            xep3[0] = xep3[1];
            xep4[0] = xep4[1]; 
            
            printf("U%f\r\n", u);
            //para el anti winup
            if(u>=100.0 | u <= -100.0){ // cambiar y poner en funcion al angulo
                u = 0.0;
                u_1 = 0.0;
                u_2 = 0.0;
                e_1 = 0.0;
                e_2 = 0.0;
            }
            
            DAC_Write(u);
                    
            u_1 = u;
            u_2 = u_1;
            e_1 = e;
            e_2 = e_1;
            X_met_1 = X_met;
            T_rad_1 = T_rad;
                     
        }
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