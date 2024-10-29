/*
 * File:   my_adc.c
 * Author: raed
 *
 * Created on February 28, 2019, 11:39 AM
 */


#include "my_adc.h"

void init_adc_no_lib(void) {
   // ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    ADCON0 = 0;
    ADCON0bits.ADON = 1; // turn adc on 

    //ADCON2 = 0b00001001; //left justified , 8 bits
    ADCON2 = 0b10001001; // right justified 10 bits

    /*   Taken from example ex4.asm
                MOVLW  B'00001101'  ; Two analog channels
                MOVWF  ADCON1
                CLRF   ADCON0
                MOVLW  B'00001001' ; Left justified
                MOVWF  ADCON2
     */

}

// reads the adc and returns a 16 bit value ( 10 bit )
// returns value 0--1023

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value; //0--1023
    ADCON0bits.CHS = channel;

    //start conversion
    ADCON0bits.GO = 1; // start conversion

    while (ADCON0bits.GO) {}; // wait until conversion is done

    raw_value = ADRESH << 8 | ADRESL; // 10 bit, need to shift the bits right

    //raw_vale = ADRESH ; //for 8 bit , use this, 
    
    return raw_value;
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
    //suppose in the above we had a temperature with sensor range -10 ---50C
    // Then we will convert as follows
    // float temp ;
    // temp = (raw_value * ( 50 - (-10)) /1023.0  + (-10.0); // parentheses used to illustrate max  and min range
    //return temp 

}












