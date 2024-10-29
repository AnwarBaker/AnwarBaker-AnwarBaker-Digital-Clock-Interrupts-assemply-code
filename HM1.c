/*
 * File:   HM1.c
 * Author: az
 *
 * Created on 22 ??????, 2024, 04:08 ?
 */

#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF

#include <xc.h>
#include <stdio.h>
#include <String.h>
#include "my_ser.h"
#include "my_adc.h"
#include "lcd_x8.h"



#define _XTAL_FREQ 4000000
char Buffer[32];
// Clock variables
volatile unsigned char seconds = 0, minutes = 0, hours = 0;



volatile bit clockMode = 0; // 0: Normal Mode, 1: Setup Mode
volatile unsigned char setupSelect = 0; // 0: Seconds, 1: Minutes, 2: Hours

// Heater and Cooler status
bit heater_status = 0;
bit cooler_status = 0;

// Analog Input variables
float temperature = 0;

// Function prototypes
void init_timer0(void);
void handle_buttons(void);
void display_lcd(void);
void handle_uart(void);
void toggle_heater(void);
void toggle_cooler(void);

// Timer interrupt

void __interrupt() ISR() {
    if (TMR0IF) {
        // Timer 0 interrupt every second
        T0CON = 0x85;       // Configure Timer0 with prescaler 1:64
        TMR0 = 45536; 
         // Reload value for 1 second delay
        if (!clockMode) { // Normal mode
            seconds++;
            if (seconds >= 60) {
                seconds = 0;
                minutes++;
                if (minutes >= 60) {
                    minutes = 0;
                    hours++;
                    if (hours >= 24) {
                        hours = 0;
                    }
                }
            }
        }
        TMR0IF = 0; // Clear Timer0 interrupt flag
    }

    // Handle INT1 (RB1) - Toggle between Normal and Setup mode
    if (INT1IF) {
        clockMode = !clockMode;
        INT1IF = 0; // Clear interrupt flag
    }

    // Handle INT2 (RB2) - Toggle between setting Seconds, Minutes, Hours in Setup Mode
    if (INT2IF && clockMode) {
        setupSelect = (setupSelect + 1) % 3;
        INT2IF = 0; // Clear interrupt flag
    }

    // Handle INT0 (RB0) - Toggle cooler
    if (INT0IF) {
        toggle_cooler();
        INT0IF = 0; // Clear interrupt flag
    }
}




/*
void setupPorts(void) {
    // Set all AN pins as analog (or configure as needed)
    ADCON1 = 0x0E;  // Configure all pins as digital except AN0 (configure based on your setup)

    // Configure port directions
    TRISB = 0xFF;   // Set PORTB as input (for buttons and interrupts)
    TRISC = 0x00;   // Set PORTC as output (for heater, cooler control)
    TRISA = 0xFF;   // Set PORTA as input (for analog inputs)
    TRISD = 0x00;   // Set PORTD as output (for LCD)
    TRISE = 0x00;   // Set PORTE as output (for LCD)

    // Initialize outputs
    LATC = 0x00;    // Ensure all outputs are low (cooler and heater off)
    LATD = 0x00;    // LCD initialized with low values
    LATE = 0x00;    // LCD pins initialized with low values
}
*/

void setupPorts(void) {
    // Set AN2 as an analog input, others as digital
    ADCON1 = 0x09;  // Configure AN2 as analog, others as digital

    // Configure port directions
    TRISB = 0xFF;   // Set PORTB as input (for buttons and interrupts)
    TRISC = 0x00;   // Set PORTC as output (for heater, cooler control)
    TRISA = 0xFF;   // Set PORTA as input (for analog inputs)
    TRISD = 0x00;   // Set PORTD as output (for LCD)
    TRISE = 0x00;   // Set PORTE as output (for LCD)

    // Initialize outputs
    LATC = 0x00;    // Ensure all outputs are low (cooler and heater off)
    LATD = 0x00;    // LCD initialized with low values
    LATE = 0x00;    // LCD pins initialized with low values
}



// Timer0 setup
void init_timer0(void) {
    T0CON = 0x87;       // Configure Timer0
                        // 8-bit timer, prescaler 1:256
                        // TMR0ON = 1 (enable Timer0)
                        // PSA = 0 (prescaler is assigned)
     T0CON = 0x85;       // Configure Timer0 with prescaler 1:64
               // T0PS = 111 (prescaler 1:256)
    TMR0 = 45536;     // Initial value for 1 second overflow
    TMR0IE = 1;         // Enable Timer0 interrupt
    GIE = 1;            // Enable global interrupts
    PEIE = 1;           // Enable peripheral interrupts
}



unsigned char wait_for_byte(void) {
    while (!is_byte_available()) {
        // Waiting for input from UART
    }
    return read_byte_no_lib();
}

void handle_uart(void) {
    if (is_byte_available()) {
        unsigned char cmd = read_byte_no_lib();
        char buffer[16];
        switch (cmd) {
            case 't': // Read time
                sprintf(buffer, "\n%02u:%02u:%02u", hours, minutes, seconds);
                send_string_no_lib((const unsigned char *)buffer);
                break;
                
            case 'T': // Read temperature
                sprintf(buffer, "\nT: %.1f", temperature);
                send_string_no_lib((const unsigned char *)buffer);
                break;
                
            case 's': // Read heater and cooler status
                sprintf(buffer, "\nHeater: %s\nCooler: %s", heater_status ? "ON" : "OFF", cooler_status ? "ON" : "OFF");
                send_string_no_lib((const unsigned char *)buffer);
                break;
                
            case 'w': // Write time in the format "whh:mm:ss"
            {
                unsigned char h1, h2, m1, m2, s1, s2,c1,c2;

                // Prompt and wait for hours input
                
                h1 = wait_for_byte() - '0'; // Read first digit of hours
                h2 = wait_for_byte() - '0'; // Read second digit of hours
                
                
                
                 c1=wait_for_byte();
                 
                 
                // Prompt and wait for minutes input
                
                m1 = wait_for_byte() - '0'; // Read first digit of minutes
                m2 = wait_for_byte() - '0'; // Read second digit of minutes
                
                
               
                 c2=wait_for_byte();
                
                
                // Prompt and wait for seconds input
               
                s1 = wait_for_byte() - '0'; // Read first digit of seconds
                s2 = wait_for_byte() - '0'; // Read second digit of seconds

                // Combine digits to form the actual time values
                hours = h1 * 10 + h2;
                minutes = m1 * 10 + m2;
                seconds = s1 * 10 + s2;

                // Validate time values
                if (hours < 24 && minutes < 60 && seconds < 60) {
                    sprintf(buffer, "\nTime Set: %02u:%02u:%02u", hours, minutes, seconds);
                    send_string_no_lib((const unsigned char *)buffer);
                } else {
                    send_string_no_lib("\nInvalid time entered!\n");
                }
            }
            break;
            
            default:
                break;
        }
    }
}

// Heater toggle (RB4)

void toggle_heater(void) {
    if (RB4 == 0) {
        heater_status = !heater_status;
        RC5 = heater_status; // Control RC5
        __delay_ms(300);     // Debounce
    }
}

// Cooler toggle (INT0)
void toggle_cooler(void) {
    cooler_status = !cooler_status;
    RC2 = cooler_status; // Control RC2
}

// Handle buttons for setup mode (RB2: increase, RB3: decrease)
// Handle buttons for setup mode (RB5: increase, RB3: decrease)
void handle_buttons(void) {
    if (clockMode) { // Only handle buttons in Setup Mode
        if (RB5 == 0) {  // Use RB5 to increment
            __delay_ms(200); // Debounce
            if (setupSelect == 0) seconds = (seconds + 1) % 60;
            else if (setupSelect == 1) minutes = (minutes + 1) % 60;
            else if (setupSelect == 2) hours = (hours + 1) % 24;
        }
        if (RB3 == 0) {  // Use RB3 to decrement
            __delay_ms(200); // Debounce
            if (setupSelect == 0) seconds = (seconds == 0) ? 59 : seconds - 1;
            else if (setupSelect == 1) minutes = (minutes == 0) ? 59 : minutes - 1;
            else if (setupSelect == 2) hours = (hours == 0) ? 23 : hours - 1;
        }
    }
}


// Display LCD
void display_lcd(void) {
    lcd_gotoxy(1, 1); // Line 1: Time and Temperature
    sprintf(Buffer, "%02u:%02u:%02u    %.1fC", hours, minutes, seconds, temperature);
    lcd_puts(Buffer);

    lcd_gotoxy(1, 2); // Line 2: Cooler and Heater status
    sprintf(Buffer, "C: %s  H: %s", cooler_status ? "ON " : "OFF", heater_status ? "ON " : "OFF");
    lcd_puts(Buffer);

    lcd_gotoxy(1, 3); // Line 3: Mode and Setup selection
    if (clockMode) {
        if (setupSelect == 0) lcd_puts("Setup: Seconds");
        else if (setupSelect == 1) lcd_puts("Setup: Minutes");
        else if (setupSelect == 2) lcd_puts("Setup: Hours  ");
    } else {
        
        lcd_puts("Normal    Mode");
    }

    lcd_gotoxy(1, 4); // Line 4: Names
    lcd_puts("Anwar      Abood");
}








// Main function
void main(void) {
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    init_timer0();

    while (1) {
        // Read analog input for temperature
        temperature = read_adc_voltage(2) * 100; // Convert voltage to temperature
        handle_buttons();
        display_lcd();
        handle_uart(); // Check UART communication
        toggle_heater(); // Check heater button
    }
}




