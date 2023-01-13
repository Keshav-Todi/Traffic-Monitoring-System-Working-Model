/* 
 * File:   main.c
 * Author: Ash Pylypow 1685559
 *
 * Created on November 24, 2022, 9:34 PM
 */


//#define F_CPU 8000000UL
#define F_CPU 14745600UL


#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>


#define BAUD_RATE 19200
// Taken from ATmega328p datasheet:
#define UBRR F_CPU/16/BAUD_RATE-1

//#define MAX_RANGE 70.0f // 400.0f
// Speed of sound in cm/s
#define SPEED_OF_SOUND 34300.0f


struct TIME
{
    uint8_t hours; // 24-hour
    uint8_t minutes;
    uint8_t seconds;
    
    uint16_t milliseconds;
};

// Timter0 will interrupt every 10ms, 1 loop duration is 100ms,
// Count up to 10 interrupts for 1 loop duration.
#define MAX_INT_COUNT 10

volatile uint8_t interrupt_count = 0;
volatile uint8_t pacing_ready = 0;

volatile struct TIME time;


//ISR(TIMER1_COMPA_vect)
ISR(TIMER0_COMPA_vect)
{
    interrupt_count++;
    if (interrupt_count >= 10)
    {    
        pacing_ready = 1;
        interrupt_count = 0; // Reset count
    }
    
    UpdateTime(&time);
}


// Adapted from the ATmega329P manual, p. 186
void UART_Transmit(char* str)
{
    // NOTE: assuming a well-formed string
    while (*str != '\0') // Continue while not at end of string
    {
        // Wait for empty buffer
        while (!(UCSR0A & (1 << UDRE0))) ;
        
        UDR0 = *str; // Send current character
        str++; // Move pointer to next character
    }
}

// Adapted from the ATmega329P manual, p. 189
void UART_Receive(char* rec_str)
{
    // NOTE: no checking for buffer overflow of rec_str!!!
    char* ptr = rec_str; // Temp pointer to avoid invalidating the passed in pointer from caller
    
    while (1)
    {
        while (!(UCSR0A & (1 << RXC0))) ; // Wait to receive data
    
        *ptr = UDR0;
        
        if (*ptr == '\n') // Assume '\n' will be end of message
        {
            ptr++;
            *ptr = '\0'; // Add end of string character.
            return;
        }
        
        ptr++;
    }
}


void GetTime(struct TIME* time)
{
    // No error checking provided for input!
    char str[16];
    UART_Transmit("Hours: \r\n");
    UART_Receive(str);
    
    time->hours = strtol(str, NULL, 10);
    
    UART_Transmit("Minutes: \r\n");
    UART_Receive(str);
    
    time->minutes = strtol(str, NULL, 10);
    
    // Start seconds at 0
    time->seconds = 0;
    time->milliseconds = 0;
}


void UpdateTime(struct TIME* time)
{
    // Update each portion of the time as necessary
    time->milliseconds += 10; //100;
    
    if (time->milliseconds >= 1000)
    {
        time->seconds++;
        time->milliseconds -= 1000;
        
        if (time->seconds >= 60)
        {
            time->minutes++;
            time->seconds = 0;
            
            if (time->minutes >= 60)
            {
                time->hours++;
                time->minutes = 0;
                
                if (time->hours >= 24)
                {
                    time->hours = 0;
                }
            }
        }
    }
}


void LogData(struct TIME* time, float vel, uint16_t numObjects)
{
    static char str[128] = "";
    
    sprintf(&str, "%d:%d:%d:%d %.1f cm/s  %u\r\n", time->hours, time->minutes, time->seconds, time->milliseconds / 100, vel, numObjects);
    
    UART_Transmit(str);
}


void GetDist(float* dist)
{
    static uint16_t start_time = 0;
    static uint16_t end_time = 0;
    static uint16_t time; 
    static float TICK_LENGTH = 1.0f / F_CPU; //14745600.0f; // In seconds
    
    TCCR1B |= (1 << ICES1); // Capture time on rising edge (when sonar pulse begins)
    TIFR1 |= (1 << ICF1); // Clear input capture flag (not needed?)
        
    // Make sure we have a no pulse first
    PORTD &= ~(1 << PD7); // Make trigger low
    _delay_us(5);
    // Send a 10us pulse to the ultrasonic sensor
    PORTD |= (1 << PD7); // Begin trigger pulse
    _delay_us(10);
    PORTD &= ~(1 << PD7); // End trigger pulse  
    
    // Wait for pulse signal to begin (when the echo pin goes high).
    while (!(TIFR1 & (1 << ICF1))) ; 

    start_time = TCNT1;
 
    TIFR1 |= (1 << ICF1); // Clear input capture flag
    TCCR1B &= ~(1 << ICES1); // Capture on falling edge (when sonar pulse ends)
    
    // Wait for pulse signal to end (when echo pin goes low).
    while (!(TIFR1 & (1 << ICF1))) ;

    TIFR1 |= (1 << ICF1); // Clear input capture flag

    while ((TIFR1 & (1 << ICF1))) ; // Wait for echo

    end_time = TCNT1;
    
    // Calculate distance (assumes time taken is mot more than 1 overflow in duration):
    if (end_time < start_time)
    {
        time = 65536 + end_time - start_time;
    }
    else
    {
        time = end_time - start_time;
    }
    
    *dist = (time * TICK_LENGTH * SPEED_OF_SOUND) / 2.0f;
}


uint8_t GetSpeed(float* speed)
{ 
    const float MAX_DIST = 40.0f; // in cm
    float dist1;
    float dist2;
    const float time_between_dist = 10.0f; // 10 ms.
    
    // Speed is given by change in distance / change in time.
    
    GetDist(&dist1);
    
    // Assume no object present if distance is >= MAX_DIST
    if (dist1 > MAX_DIST)
        return 0;
    
    _delay_ms(time_between_dist);
    
    GetDist(&dist2);
    
    if (dist2 > MAX_DIST) // Object may have gone out of range before getting second distance reading
        return 0;
    
    *speed = fabs(dist2 - dist1) / (time_between_dist / 1000); // Make sure we have cm/s
    
    return 1;
}


int main()
{
    // Set clock division factor to 1
    CLKPR = (1 << CLKPCE);
    CLKPR = 0; // No clock division (factor 1)
    
    // USART setup:
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Enable USART receive and transmit
    // Initial values happen to be 8N1 settings
    UBRR0 = UBRR;
  
    // Paced loop timer uses Timer0
    TCCR0A = (1 << WGM01); // Disengage OCR0A and OCR0B, apply mode 2 
    TCCR0B = (1 << CS02) | (1 << CS00); // Timer prescaler of 1024
    OCR0A = (144 - 1); // A count of 144 will be 10ms.
    TIMSK0 = (1 << OCIE0A); // Enable OC0A interrupt
    
    
    // For the 8-bit timer.. use 1024 divider.. gives 14400 clock
    // ticks per sec, or 144 ticks per 10 ms.. use var to count the
    // interrupts at 10ms. 10 interrupts gives 10 ms.
    
    // Ultrasonic sensor
    TCCR1B |= (1 << ICES1); // Capture on rising edge
    TCCR1B |= (1 << CS10); // Pre-scale of 10
    TCCR1B &= ~(1 << ICNC1); // Disable noise canceling
    DDRD |= (1 << DDD7); // Make PD7 an output for ultrasonic trigger
    DDRB &= ~(1 << DDB0); // Make PB0 an in input for ultrasonic echo
    PORTB |= (1 << PORTB0); // Turn on pull up for PB0
    
    GetTime(&time); // Get time from terminal
    
    sei(); // Enable interrupts
    
    
    float speed = 0.0f;
    uint16_t object_count = 0;
    uint8_t isObject = 0;
    uint8_t isObjectLastTime = 0;
    
    
    while (1)
    {
        // Check for object and if object present, get speed.
        isObject = GetSpeed(&speed);
        
        if (isObject != 0)
        {
            if (isObjectLastTime == 0)
            {
                object_count++;
            }           
            
            LogData(&time, speed, object_count);
        }
        
        isObjectLastTime = isObject;
        
        // Wait for pacing interrupt
        while (!pacing_ready) ;
        pacing_ready = 0;        
    }
    
    return 1;
}