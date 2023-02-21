/*
 * File:   communicate.c
 * Author: dtan
 *
 * Created on May 30, 2022, 12:29 AM
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC = HS        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program M
#define _XTAL_FREQ 20000000

uint16_t B_rate = 9600;

// function aTien
void UARTTx_Init();
void UART_Rx_Init();
void UART_Send(uint8_t data);

// source online
void UART_send_char(char bt);
void UART_send_string(char* st_pt);
char UART_get_char() ;
void config_UART();

uint8_t UART_receive();
uint8_t flag_trans =0;
char UART_Buffer = '\0';
char result = '\0' ;
void main(void) {
    // Configuration
    config_UART();
    
    // IO config
    TRISD0 = 1 ;
    TRISA3 = 0;
    TRISA4 = 0;
    TRISA5 = 0; 
    RA3 = 1;
    RA4 = 1;
    RA5 = 1;
    // den bao 3 va 5 co van de, dang bi chong gia tri
    
    while(1){
        if (RD0==0){
//            RA3 = 0;
            __delay_ms(100);
            if(RD0 == 1){
            UART_send_string("check\n");
            }

        }
        if (!RCIF){
            result = '\0';
        }
        if (result == 'a'){
            RA4 =0;
        }
        else{
            RA4 = 1;
        }
    }
    //baud rate setting
}
//detail function
void UART_send_char(char bt)  
{
    while(!TRMT);  // hold the program till TX buffer is free
    TXREG = bt; //Load the transmitter buffer with the received value
}

void UART_send_string(char* st_pt)
{
    while(*st_pt) //if there is a char
        UART_send_char(*st_pt++); //process it as a byte data
}

//char UART_get_char()   
//{
////    if(OERR) // check for Error 
////    {
////        CREN = 0; //If error -> Reset 
////        CREN = 1; //If error -> Reset 
////    }
//    
//    while(!RCIF);  // hold the program till RX buffer is free
//    RCIF =0;
//    return RCREG; //receive the value and send it to main function
//}

void config_UART(){
     // Baud rate configuration
    BRGH = 1; // highspeed baundrate
    SPBRG = 129;//( ( _XTAL_FREQ/16 ) / B_rate) - 1 
    // Enable Asynchronous Serial Port
    SYNC = 0;
    SPEN = 1;
    // Configure Rx-Tx pin for UART 
    TRISC6 = 1;
    TRISC7 = 1;
    // Enable UART Transmission
    TXEN = 1;
    CREN =1;
    // select 8-bit mode
    // Enable Interrupt Rx 
    RCIE = 1;
    PEIE = 1;
    GIE = 1;
    
    TX9 =0;
    RX9 =0;
}


//detail anh Tien
//void UARTTx_Init();
//void UART_Rx_Init();
//void UART_Send(uint8_t data);

//void UARTTx_Init(){
//    // Baud rate configuration
//    BRGH = 1; // highspeed baundrate
//    SPBRG = ( ( _XTAL_FREQ/16 ) / B_rate) - 1 ;
//    // Enable Asynchronous Serial Port
//    SYNC = 0;
//    SPEN = 1;
//    // Configure Rx-Tx pin for UART 
//    TRISC6 = 1;
//    TRISC7 = 1;
//    // Enable UART Transmission
//    TXEN = 1;
//}
//void UART_Send(uint8_t data){
//    while(!TRMT);
//    TXREG = data;
//}

//void UART_Rx_Init(){
//    // Baud rate configuration
//    BRGH = 1;
//    SPBRG = ( ( _XTAL_FREQ/16 ) / B_rate) - 1 ;
//    // Enable Asynchronous Serial Port
//    SYNC = 0;
//    SPEN = 1;
//    // Configure Rx-Tx pin for UART 
//    TRISC6 = 1;
//    TRISC7 = 1;
//    // Enable Interrupt Rx 
//    RCIE = 1;
//    PEIE = 1;
//    GIE = 1;
//    // Enable continuous data reception
//    CREN = 1;
//}
void __interrupt() ISR(void){
    if(RCIF == 1){
        result = RCREG;
    RCIF =0;}
}
