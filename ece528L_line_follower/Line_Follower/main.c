#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"
#include "inc/Print_Binary.h"

// Set to 1 to print the sensor data, position, and FSM state to the serial terminal
// TODO: Set to 0 (or remove the debug code in main) before the demo, since the main while loop must be empty
#define DEBUG_PRINT 1

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL 2500

// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;




// Number of consecutive 1111_1111b samples required before the T-intersection is accepted
#define T_INTERSECTION_SAMPLES 3

// Minimum time (in ms) to spin at the T-intersection before searching for the line again
// Tune this so the robot rotates past the T's crossbar (~135 degrees) before it can stop turning
#define TURN_AROUND_MIN_MS 600

// Latest data read from the reflectance sensor and its calculated position
// Written by Timer_A1_Periodic_Task and read by SysTick_Handler
static volatile uint8_t reflectance_data = 0;
static volatile int32_t line_position = 0;

// Number of consecutive samples that detected the T-intersection
static volatile uint8_t t_intersection_count = 0;

// Flag that is set once the robot has reached the T-intersection and turned around
static uint8_t t_intersection_explored = 0;

// Keeps track of how long the robot has been turning around in milliseconds
static uint32_t turn_around_ms_elapsed = 0;

// States of the Line Follower FSM
typedef enum
{
    MOVE_FORWARD,
    MOVE_TO_LEFT,
    MOVE_TO_RIGHT,
    TURN_AROUND,
    STOP
} Robot_State;

// Current state of the Line Follower FSM
static volatile Robot_State current_state = MOVE_FORWARD;

typedef enum
{
    CENTER              = 0b00011000,
    SLIGHT_LEFT_CENTER  = 0b00011100,
    SLIGHT_RIGHT_CENTER = 0b00111000,
    FAR_LEFT            = 0b00000001,
    FAR_RIGHT           = 0b10000000,
    LEFT_TURN           = 0b11111000,
    RIGHT_TURN          = 0b00011111,
    T_INTERSECTION      = 0b11111111,
    DEAD_END            = 0b00000000
} Line_Position;


void Detect_Line_Position(uint8_t reflectance_sensor_data)
{
    switch(reflectance_sensor_data)
    {
        case CENTER:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_GREEN);
        }
        break;

        case SLIGHT_LEFT_CENTER:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_YELLOW);
        }
        break;

        case SLIGHT_RIGHT_CENTER:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_PINK);
        }
        break;

        case FAR_LEFT:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_WHITE);
        }
        break;

        case FAR_RIGHT:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_SKY_BLUE);
        }
        break;

        case LEFT_TURN:
        {
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_WHITE);
        }
        break;

        case RIGHT_TURN:
        {
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_SKY_BLUE);
        }
        break;

        case T_INTERSECTION:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_BLUE);
        }
        break;

        case DEAD_END:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_RED);
        }
        break;

        default:
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_OFF);
        }
    }
}


void Timer_A1_Periodic_Task(void)
{
    // Increment Timer_A1_ms_elapsed by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 10, 20, 30, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor sensor array after 1 ms (i.e. 11, 21, 31, ...)
    if ((Timer_A1_ms_elapsed % 10) == 1)
    {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);

        reflectance_data = Reflectance_Sensor_Data;
        line_position = Reflectance_Sensor_Position(Reflectance_Sensor_Data);

        // Count consecutive T-intersection samples
        if (Reflectance_Sensor_Data == T_INTERSECTION)
        {
            if (t_intersection_count < T_INTERSECTION_SAMPLES)
            {
                t_intersection_count++;
            }
        }
        else
        {
            t_intersection_count = 0;
        }

    }
}


Robot_State Get_Next_Line_State(Robot_State state)
{
    if ((t_intersection_explored == 0) && (t_intersection_count >= T_INTERSECTION_SAMPLES))
    {
        t_intersection_explored = 1;
        turn_around_ms_elapsed = 0;
        return TURN_AROUND;
    }

    if (reflectance_data == DEAD_END)
    {

        if (t_intersection_explored == 1)
        {
            return STOP;
        }
        return state;
    }

    if ((line_position > -47) && (line_position < 47))
    {
        return MOVE_FORWARD;
    }
    else if (line_position <= -47)
    {
        return MOVE_TO_LEFT;
    }
    else
    {
        return MOVE_TO_RIGHT;
    }
}


void SysTick_Handler(void)
{
    switch(current_state)
    {
        case MOVE_FORWARD:
        {
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            current_state = Get_Next_Line_State(current_state);
        }
        break;

        case MOVE_TO_LEFT:
        {
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            current_state = Get_Next_Line_State(current_state);
        }
        break;

        case MOVE_TO_RIGHT:
        {
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            current_state = Get_Next_Line_State(current_state);
        }
        break;

        case TURN_AROUND:
        {
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            turn_around_ms_elapsed++;

            if ((turn_around_ms_elapsed >= TURN_AROUND_MIN_MS) &&
                (line_position > -47) && (line_position < 47))
            {
                current_state = MOVE_FORWARD;
            }
        }
        break;

        case STOP:
        {
            Motor_Stop();
        }
        break;

        default:
        {
            Motor_Stop();
            current_state = STOP;
        }
    }
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize the DC motors
    Motor_Init();

    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 periodic interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Enable the interrupts used by the modules
    EnableInterrupts();


    while(1)
    {

    }
}
