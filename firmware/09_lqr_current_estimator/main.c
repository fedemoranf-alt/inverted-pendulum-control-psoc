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
float X_met_1; //valor anterior de x_met
float X_vel; //derivada de x_met, velocidad del carrito
float T_rad;
float T_rad_1; //valor anterior de T_rad
float T_vel;

              
//Definicion del sistema para pelota de tenis con 0.01s, 
 float A [4][4]= {{1.0000  ,  0.0093  ,  0.0000  ,  0.0000},
    {0  ,  0.8645  ,  0.0076  ,  0.0000},
        {    0  , -0.0021  ,  1.0016  ,  0.0100},
            {0  , -0.4021  ,  0.3133  ,  1.0016}};


float B[4][1] = { {0.0001},
  {  0.0103},
 {   0.0002},
{0.0305}};

float C[2][4] = {{1,     0,     0,     0},
                {0,     0,     1,     0}};


/*
//DEFINICIONES PARA 15MS
float A[4][4] = {{1.0000 ,   0.0135  ,  0.0001 ,   0.0000},
  {       0  ,  0.8038  ,  0.0109  ,  0.0001},
 {        0  , -0.0045  ,  1.0035  ,  0.0150},
{0  , -0.5826  ,  0.4691  ,  1.0035 }};

float B[4][1] = {{0.0001},
    {  0.0149},
 {   0.0003},
{0.0442}   };

float C[2][4] = {{1,     0,     0,     0},
                {0,     0,     1,     0}};
*/
/*
//DEFINICION PARA 20MS
float A[4][4] = {{1.0000  ,  0.0174   , 0.0001   , 0.0000},
    {0  ,  0.7474  ,  0.0141  ,  0.0001},
        {    0  , -0.0079  ,  1.0063  ,  0.0200},
            { 0  , -0.7509   , 0.6245   , 1.0063}};

float B[4][1] = {{0.0002},
{0.0192},
 {   0.0006},
  {  0.0570}};

float C[2][4] = {{1,     0,     0,     0},
                {0,     0,     1,     0}};

*/

//variables del integrador
float V[] = {0.0, 0.0, 0.0};
float K1 =   -4.0979;
float K2[] = { -777.8 ,  -754.3  ,  3618.7  ,  298.9};

//estimador de actual
float xea1[] ={0.0, 0.0, 0.0}; 
float xea2[] ={0.0, 0.0, 0.0}; 
float xea3[] ={0.0, 0.0, 0.0}; 
float xea4[] ={0.0, 0.0, 0.0}; 

float z1[] ={0.0, 0.0, 0.0}; 
float z2[] ={0.0, 0.0, 0.0}; 
float z3[] ={0.0, 0.0, 0.0}; 
float z4[] ={0.0, 0.0, 0.0}; 

//PARA 10ms
float Kea [4][2] = { {0.6299  ,  0.0000},
   {46.7092   , 0.0046},
   {-0.1488  ,  0.6800},
  {-15.6703 ,  52.1803}};


float ref = 0.0;
float e = 0.0;
float e_1 = 0.0;
float e_2 = 0.0;
float u = 0.0;
float u_1 = 0.0;
float u_2 = 0.0;

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
        if(flag == '0'){ 
            
        }
        
        if(flag == '1'){ 
           
             QuadDec_T_SetCounter(0);
            QuadDec_X_SetCounter(0);
            u = 0.0;
            X_met = 0.0;
            T_rad = 0.0;
            
            xea1[0] = 0;
            xea2[0] = 0; 
            xea3[0] = 0;
            xea4[0] = 0; 
            
            V[1] = 0;   
            V[0] = 0;
            
          /*  //esto se hace porque primero necesitamos apretar la planta y luego realizar la primera medicion, la y en el estimador actual es y(k+1), es decir nos 
            //adelantamos un paso a la medida para tener y(k+1) al medir con el adc luego
            u = ref - (K[0]*xea1[0] + K[1]*xea2[0]);
            DAC_Write(u); */
            
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
           
            //se calcula el integrador
            V[1] = V[0] + ref - T_rad;   
            V[0] = V[1];
            
            //se calcula el esfuerzo de control
            //u = K1*V[0] - (K2[0]*xea1[0] + K2[1]*xea2[0] + K2[2]*xea3[0] + K2[3]*xea4[0]);
            
            u = K1*V[0] - (K2[0]*xea1[0] + K2[2]*xea3[0] );
            //se calcula el estimador de estados actual
            
            z1[1] = (A[0][0]*xea1[0] + A[0][1]*xea2[0] + A[0][2]*xea3[0] + A[0][3]*xea4[0]) + B[0][0]*u;
            
            z2[1] = (A[1][0]*xea1[0] + A[1][1]*xea2[0] + A[1][2]*xea3[0] + A[1][3]*xea4[0]) + B[1][0]*u;
            
            z3[1] = (A[2][0]*xea1[0] + A[2][1]*xea2[0] + A[2][2]*xea3[0] + A[2][3]*xea4[0]) + B[2][0]*u;
            
            z4[1] = (A[3][0]*xea1[0] + A[3][1]*xea2[0] + A[3][2]*xea3[0] + A[3][3]*xea4[0]) + B[3][0]*u;
            
            xea1[1] =   z1[1] +
                        Kea[0][0]*(T_rad - (C[0][0]*z1[1] + C[0][1]*z2[1] + C[0][2]*z3[1] + C[0][3]*z4[1])) + 
                        Kea[0][1]*(X_met - (C[1][0]*z1[1] + C[1][1]*z2[1] + C[1][2]*z3[1] + C[1][3]*z4[1]));
            
            xea2[1] =   z2[1] +
                        Kea[1][0]*(T_rad - (C[0][0]*z1[1] + C[0][1]*z2[1] + C[0][2]*z3[1] + C[0][3]*z4[1])) + 
                        Kea[1][1]*(X_met - (C[1][0]*z1[1] + C[1][1]*z2[1] + C[1][2]*z3[1] + C[1][3]*z4[1]));
                        
            xea3[1] =   z3[1]+ 
                        Kea[2][0]*(T_rad - (C[0][0]*z1[1] + C[0][1]*z2[1] + C[0][2]*z3[1] + C[0][3]*z4[1])) + 
                        Kea[2][1]*(X_met - (C[1][0]*z1[1] + C[1][1]*z2[1] + C[1][2]*z3[1] + C[1][3]*z4[1]));

            xea4[1] =   z4[1] + 
                        Kea[3][0]*(T_rad - (C[0][0]*z1[1] + C[0][1]*z2[1] + C[0][2]*z3[1] + C[0][3]*z4[1])) + 
                        Kea[3][1]*(X_met - (C[1][0]*z1[1] + C[1][1]*z2[1] + C[1][2]*z3[1] + C[1][3]*z4[1]));
            
            xea1[0] = xea1[1];
            xea2[0] = xea2[1]; 
            xea3[0] = xea3[1];
            xea4[0] = xea4[1]; 
            
            printf("U%f\r\n", u);
            
            //para el anti winup
            if(u>=200.0 | u <= -200.0){ // cambiar y poner en funcion al angulo
                u = 0.0;
                xea1[0] = 0;
                xea2[0] = 0; 
                xea3[0] = 0;
                xea4[0] = 0; 
                V[1] = 0;   
                V[0] = 0;
            }
            
            DAC_Write(u);
                     
        }
        
        if(flag == '3'){ 
            
            isr_1_Disable();
            DAC_Write(0);
            flag = '0';
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
            if(rxData == 'P'){
             flag = '3';   
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