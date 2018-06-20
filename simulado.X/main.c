/*
 * File:   main.c
 * Author: Luis Felipe Kunzler
 *
 * Created on 11 de Junho de 2018, 15:37
 */

// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define LED1 PORTCbits.RC0

#define _XTAL_FREQ 20000000

volatile uint8_t cont_decseg = 0;
volatile uint8_t cont_seg = 0;
volatile short ativo = 0;

void set_dc_percent(uint8_t percent) 
{
    /*
     * Como foi especificado no projeto, o dutycicle inicial deve ser de 50%
     * Como o Tpwm é de 1/5000 = 200us, a saida do ccp2 deve ficar em estado
     * logico alto por 100us. Utilizando a eq 15-2 do datasheet:
     * 100us = (CCPR2L:CCP2CON<5,4>) * 1/20MHz * 4
     * (CCPR2L:CCP2CON<5,4>) = 500 = 0x01F4 = 0b0111110100, onde
     * CCPR2L = 01111101;
     * CCP2CON<5,4> = 00;
     */
    uint16_t dc = percent*10; // multiplica por 10
    
    CCP2CONbits.DC2B = dc & 0x0003; // apenas os bits menos significativos
    CCPR2L = dc>>2; // ignora os 2 bits menos significativos    
}

void interrupt ISR()
{
    if (INT0IE && INT0IF) {
        set_dc_percent(50);
        INT0IF = 0; // reconheceu a interrupcao
    }
}

void interrupt low_priority ISR_low()
{
    if (TMR1IF && TMR1IE) { // estouro do timer 1
        // resseta o timer para dar 100ms
        TMR1H = 0x0B;
        TMR1L = 0xDC;
        LED1 = !LED1;
        
        cont_decseg++;
        cont_seg += cont_decseg/10;
        cont_decseg = cont_decseg % 10;        
        
        TMR1IF = 0; // limpa flag de interrupcao
    }
    
    if (CCP1IF && CCP1IE) { 
        if (CCP1CONbits.CCP1M == 0x05) { // se estava em rising edge
            // quer dizer que o evento acabou
            ativo = 0; // sinaliza que o processo foi desativado
            uint8_t dc = cont_seg*10 + cont_decseg;
            set_dc_percent(dc > 100 ? 100 : dc);
        }
        else if (CCP1CONbits.CCP1M == 0x04) { // falling edge
            // quer dizer que o evento iniciou
            ativo = 1; // processo esta ativo
            // zera os contadores de tempo
            cont_decseg = 0;
            cont_seg = 0;
            PORTD = 0xFF; // ao recomecar o novo processo, limpa os leds
        }        
        CCP1CONbits.CCP1M ^= 0x01; // inverte o sentido da borda para int        
        CCP1IF = 0; // sinaliza que a interrupcao foi tratada
    }
}

void main(void)
{
    ADCON1 = 0x0F; // all digital    
    TRISC = 0xFC; // RC0 e RC1 saida
    TRISD = 0x00; // portd saida
    PORTD = 0xFF; // leds desligados  
    /*
     * Para calcularmos a frequencia do PWM, definimos valores de PR2 e Prescaler
     * Como sabemos que a frequencia deve ser de 5kHz, na equacao...
     * Tpwm =  (PR2+1) * 4 * Tosc * PreScalerTMR2 = 1/5000
     * PR2 = 249, Tosc = 1/20MHz e PreScaler do TMR2 = 4.
     */
    PR2 = 249; // valor target do timer2
    TMR2 = 0; // zera o timer

    /*
     * T2CON = [x, pos3, pos2, pos1, pos0, tmr2on, pre1, pre0];
     * T2CON = 0 0000 1 01;
     */
    T2CON = 0x05; // pos = 1:1, tmr = on, preS = 1:4

    /* 
     * IPEN faz parte do registrador RCON (Reset Control Register)     
     */
    IPEN = 1; // ativa diferenca de prioridade entre interrupcoes

    /*
     * GIEH e PEIE funcionam como habilitacao do grupo de prioridades quando IPEN
     * está em 1. Fazem parte do INTCON
     */
    GIEH = 1; // ativa interrupcoes de alta prioridade
    //GIEL = 1; // ativa interrupcoes globais de baixa prioridade
    PEIE = 1; // ativa interrupcos de baixa prioridade

    // PIR = Peripheral Interrupts Flags
    // PIE = Peripheral Interrupts Enable
    //TMR2IE = 1; //TMR2 to PR2 match interruption enabled

    /*
     * O DutyCicle é definido pelos registradores CCPRxL e CCPxCON<5-4>
     *      CCPRxL          CCPxCON,5   CCPxCON,4
     * 0 0 0 0 0 0 0 0          0           0
     *
     * Para habilitar o PWM no CCP2, escrevemos nos 4 bits menos significativos
     * do reg CCP2CON: [x, x, DCB1, DCB0, M3, M2, M1, M0];
     * Para PWM, M3 e M2 em 1, o resto nao importa. LSB1 e LSB2 definem o estado
     * inicial do duty
     */
    CCP2CON = 0x0F; // PWM no CCP2 ativo, bits menos significativos do DC em 0

    /*
     * Como foi especificado no projeto, o dutycicle inicial deve ser de 50%
     * Como o Tpwm é de 1/5000 = 200us, a saida do ccp2 deve ficar em estado
     * logico alto por 100us. Utilizando a eq 15-2 do datasheet:
     * 100us = (CCPR2L:CCP2CON<5,4>) * 1/20MHz * 4
     * (CCPR2L:CCP2CON<5,4>) = 500 = 0x01F4 = 0b0111110100, onde
     * CCPR2L = 01111101;
     * CCP2CON<5,4> = 00;
     */
    //CCPR2L = 0x7D;
    //CCP2CONbits.DC2B = 0;
    set_dc_percent(50);
    /* fim do pwm */

    /* inicio do timer1 e capture */
    /*
     * como precisamos de um periodo de estouro de 100ms
     * Tosc = 1/20MHz
     * PreScaler TMR1 = 8
     * Inicio do timer = 3036
     */
    // CONFIGURA O TIMER1
    T1CONbits.RD16 = 1; // usa 16 bits
    T1CONbits.T1RUN = 0; // nao usa o clock auxiliar
    T1CONbits.T1CKPS1 = 1; // post em 8
    T1CONbits.T1CKPS0 = 1;
    T1CONbits.T1OSCEN = 0; // desativa o oscilador do tmr1
    T1CONbits.TMR1CS = 0; // osc e baseado no ciclo de maquina (fosc/4)
    T1CONbits.TMR1ON = 1; // tmr1 ativo

    TMR1IE = 1; // ativa as interrupcoes do timer 1
    TMR1IP = 0; // timer1 em baixa prioridade 
    
    // escreve 3036, para gerar uma interrupcao a cada 100ms
    TMR1H = 0x0B;
    TMR1L = 0xDC;
    
    // CONFIGURACAO CCP1 COMO CAPTURE
    // como o sensor gera borda de descida no primeiro marco e borda de descida 
    // no segundo, configuramos ele inicialmente para capture com falledge
    CCP1CON = 0x04; // capture mode every falling edge
    T3CONbits.T3CCP2 = 0; // tmr1 para ccp2 quando capture/compare
    T3CONbits.T3CCP1 = 0; // tmr1 para ccp1 quando capture/compare
    
    PIE1bits.CCP1IE = 1; // habilita interrupcoes pelo ccp1    
    IPR1bits.CCP1IP = 0; // baixa prioridade para interrupcoes do ccp1
    /* fim da configuracao do capture */
    
    /* configuracao do pino INT0 */
    INTEDG0 = 0; // interrupt on falling edge
    INT0IE = 1; // habilita interrupcao no pino    
    
    uint8_t segundos = 0;
    
    while (1) {
        if (cont_seg > segundos && cont_seg < 8 && ativo) {
            PORTD <<= 1;
            segundos = cont_seg; 
        }        
    }

    return;
}
