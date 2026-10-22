/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Sam Hamed
 *
 */

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

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
static Robot_State current_state = MOVE_FORWARD;

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

/**
 * @brief Indicates the detected line maze possibility using the built-in red LED and the RGB LED.
 *
 * This function uses a switch statement to map each value read from the reflectance sensor
 * to a specific LED1 and RGB LED combination. The LEDs are turned off for any other value.
 *
 * @param reflectance_sensor_data The 8-bit value read from the reflectance sensor array.
 *
 * @return None
 */
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

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 1 kHz.
 *
 * The Timer_A1_Periodic_Task function samples the reflectance sensor array. It starts a reading
 * every 10 ms and finishes the reading 1 ms later. The result is stored in reflectance_data,
 * converted into a position with Reflectance_Sensor_Position(), and displayed on the LEDs
 * with Detect_Line_Position(). It also counts how many consecutive samples have detected
 * the T-intersection so that a single noisy reading does not trigger a turn around.
 *
 * @param None
 *
 * @return None
 */
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

        // Store the latest sample and its position for the FSM in SysTick_Handler
        reflectance_data = Reflectance_Sensor_Data;
        line_position = Reflectance_Sensor_Position(Reflectance_Sensor_Data);

        // Count consecutive T-intersection samples
        if (Reflectance_Sensor_Data == T_INTERSECTION)
        {
            if (t_intersection_count < 255)
            {
                t_intersection_count++;
            }
        }
        else
        {
            t_intersection_count = 0;
        }

        Timer_A1_ms_elapsed = 0;
    }
}

/**
 * @brief Determines the next line following state based on the latest reflectance sensor data.
 *
 * This function checks for the T-intersection and the dead end first, since a T-intersection
 * (1111_1111b) has a calculated position of 0 and would otherwise be treated as the center.
 * Then, it uses the line position to select MOVE_FORWARD, MOVE_TO_LEFT, or MOVE_TO_RIGHT
 * as shown in the Line Follower FSM (Figure 4.6-1).
 *  - T-intersection (before it has been explored): TURN_AROUND
 *  - Dead end (after the T-intersection has been explored): STOP
 *  - Dead end (before the T-intersection has been explored): keep the current state
 *  - -47 < Position < 47: MOVE_FORWARD
 *  - Position <= -47: MOVE_TO_LEFT
 *  - Position >= 47: MOVE_TO_RIGHT
 *
 * @param state The current state of the FSM.
 *
 * @return The next state of the FSM.
 */
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
        // The line has ended. Stop if the robot is returning from the T-intersection.
        // Otherwise, the line was briefly lost, so keep doing the previous action.
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

/**
 * @brief Interrupt service routine for the SysTick timer, which runs the Line Follower FSM.
 *
 * The SysTick_Handler is executed every 1 ms. It uses a switch statement on current_state
 * to drive the motors for the current state and then determines the next state.
 *  - MOVE_FORWARD:  Both motors move forward
 *  - MOVE_TO_LEFT:  The robot rotates left to bring the line back to the center
 *  - MOVE_TO_RIGHT: The robot rotates right to bring the line back to the center
 *  - TURN_AROUND:   The robot spins in place for at least TURN_AROUND_MIN_MS,
 *                   then returns to line following once the line is centered again
 *  - STOP:          The motors are stopped after the robot reaches the dead end
 *
 * @param None
 *
 * @return None
 */
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

            // Wait until the robot has rotated past the T's crossbar,
            // then return to line following once the line is centered
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
