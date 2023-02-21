/*
 * File:   BTL.c
 * Author: dtan
 *
 * Created on June 1, 2022, 4:02 AM
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC = HS  // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF  // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF  // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF  // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF   // Flash Program M

#define _XTAL_FREQ 20000000

//======================================================
//--------------------[ Servo ]-------------------------
#define WritePosOpenServo       RD0
#define OpenServo               RD1
#define SelectServo             RD2
#define WriteSelectServo        RD3

uint8_t cnt = 0;
uint8_t pos = 2;
/*----------*/
void Servo_Init();
/*----------*/

//======================================================
//--------------------[ I2C ]---------------------------
void I2C_Master_Init();
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_Write(unsigned char data);
/*----------*/

//======================================================
//--------------------[ LCD ]---------------------------
#define LCD_CLEAR               0x01
#define LCD_RETURN_HOME         0x02
#define LCD_TURN_ON             0x0C
#define LCD_TURN_OFF            0x08
#define LCD_SWAP                RD4

unsigned char RS, i2c_add, BackLight_State = 0x08;
uint8_t LCD_SWAP_OLL = 0;
/*----------*/
void LCD_Init(unsigned char I2C_Add);
void IO_Expander_Write(unsigned char Data);
void LCD_Write_4Bit(unsigned char Nibble);
void LCD_CMD(unsigned char CMD);
void LCD_Write_Char(char Data);
void LCD_Write_String(char *Str);
void LCD_Set_Cursor(unsigned char ROW, unsigned char COL);
void LCD_Clear();
/*----------*/

void main(void)
{

    Servo_Init();
    I2C_Master_Init();
    LCD_Init(0x27); // Initialize LCD module with I2C address = 0x4E

    while (1)
    {
        if (OpenServo == 1)
        {
            pos = 20;
        }
        else
        {
            pos = 10;
        }
        WriteSelectServo = SelectServo;

        if (LCD_SWAP == 0 && LCD_SWAP != LCD_SWAP_OLL)
        {
            LCD_Clear();
            LCD_Set_Cursor(1, 1);
            LCD_Write_String("SP1 SP2 SP3");
            LCD_Set_Cursor(2, 1);
            LCD_Write_String("0  .9  .10 ");
            LCD_SWAP_OLL = 0;
        }
        else
        {
            if (LCD_SWAP != LCD_SWAP_OLL)
            {
                LCD_Clear();
                LCD_Set_Cursor(1, 1);
                LCD_Write_String("Nam Lee");
                LCD_Set_Cursor(2, 1);
                LCD_Write_String("kakaka");
                LCD_SWAP_OLL = 1;
            }
        }
        WriteSelectServo = SelectServo;

    }; // 20M->5M->2.5M-> 2500 trong 1ms -> 250 0.1ms
}

//======================================================
//--------------------[ Servo ]-------------------------
void Servo_Init()
{
    // config timer 0
    PSA = 0; // chon bo chia truoc cho timer 0

    PS2 = 0;
    PS1 = 0;
    PS0 = 0; // chon bo chia truoc 2

    T0CS = 0; // chon nguon xung clock noi

    GIE = 1;  // cho phep ngat
    T0IE = 1; // cho phep ngat timer 0
    T0IF = 0; // ghi gia tri co ngat = 0

    TRISD = 0x16; // OUT D0,3 set OUTPUT, D1,2,4 INPUT
    PORTD = 0x01; // D0=0
}

void __interrupt() isr1()
{
    if (T0IF == 1) // phat hien co bao ngat
    {
        TMR0 = 5;
        T0IF = 0; // dua co bao ngat ve gia tri 0
        cnt++;

        if (cnt == pos) // khi timer dem du time
        {
            WritePosOpenServo = ~WritePosOpenServo;
        }

        if (cnt == 200 - pos) // khi timer dem du time 200 cnt = 20ms
        {
            WritePosOpenServo = ~WritePosOpenServo;
            cnt = 0; // reset gia tri cnt
        }
    }
}

//======================================================
//--------------------[ I2C ]---------------------------
void I2C_Master_Init()
{
    SSPCON = 0x28; // dat pic la master, clock = FOSC/(4 * (SSPADD + 1))
    SSPCON2 = 0x00;
    SSPSTAT = 0x00;
    SSPADD = ((_XTAL_FREQ / 4) / 100000) - 1; // I2C_BaudRate           100000

    TRISC3 = 1; // SCL
    TRISC4 = 1; // SDA
}

void I2C_Master_Wait()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)) // cho den khi du lieu truyen xong
        ;
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SEN = 1; // bat dau su dung i2C
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    PEN = 1; // ket thuc su dung i2C
}

void I2C_Master_Write(unsigned char data) // ghi du lieu vao bo nho dem SSPBUF
{
    I2C_Master_Wait();
    SSPBUF = data; // khi nhan duoc 1 byte hoan chinh, bit SSIF == 1
    while (!SSPIF)
        ; // doi du lieu day
    SSPIF = 0;
}

//======================================================
//--------------------[ LCD ]---------------------------
void LCD_Init(unsigned char I2C_Add)
{
    i2c_add = I2C_Add;
    IO_Expander_Write(0x00);
    __delay_ms(5);
    
    LCD_CMD(LCD_RETURN_HOME);
    __delay_ms(5);
    LCD_CMD(0x20 | (2 << 2));
    __delay_ms(5);
    LCD_CMD(LCD_TURN_ON);
    __delay_ms(50);
    LCD_CMD(LCD_CLEAR);
    __delay_ms(50);
}

void IO_Expander_Write(unsigned char Data)
{
    I2C_Master_Start();
    I2C_Master_Write(i2c_add);
    I2C_Master_Write(Data | BackLight_State);
    I2C_Master_Stop();
}

void LCD_Write_4Bit(unsigned char Nibble)
{
    // Get The RS Value To LSB OF Data
    Nibble |= RS;
    IO_Expander_Write(Nibble | 0x04);
    IO_Expander_Write(Nibble & 0xFB);
    //	__delay_us(50);
}

void LCD_CMD(unsigned char CMD)
{
    RS = 0; // Command Register Select
    LCD_Write_4Bit(CMD & 0xF0);
    LCD_Write_4Bit((CMD << 4) & 0xF0);
}

void LCD_Write_Char(char Data)
{
    RS = 1; // Data Register Select
    LCD_Write_4Bit(Data & 0xF0);
    LCD_Write_4Bit((Data << 4) & 0xF0);
}

void LCD_Write_String(char *Str)
{
    for (int i = 0; Str[i] != '\0'; i++)
        LCD_Write_Char(Str[i]);
}

void LCD_Set_Cursor(unsigned char ROW, unsigned char COL)
{
    switch (ROW)
    {
    case 2:
        LCD_CMD(0xC0 + COL - 1); //
        break;
    // Case 1
    default:
        LCD_CMD(0x80 + COL - 1); // 1100.0000 D7=1 co dinh, hang 1 bat dau tu 0x00 nen tro thanh 0x80
        break;
    }
}

void LCD_Clear()
{
    LCD_CMD(LCD_CLEAR);
    	__delay_us(40);
}
