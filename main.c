#include <msp430.h>
#include "circular_buffer.h"
#include "serial_protocol.h"
#include <math.h>

/*-- Global Variables --*/
uint8_t __UART_RX_BUFFER_MEMORY[64 + 1];
CircularBuffer UART_RX_BUFFER = {
    .data = __UART_RX_BUFFER_MEMORY,
    .head = 0,
    .tail = 0,
    .size = sizeof(__UART_RX_BUFFER_MEMORY) / sizeof(__UART_RX_BUFFER_MEMORY[0])};

/* Stepper Global States */
atomic_bool STEPPER_DIR = false;
atomic_bool STEPPER_EXECUTION_PENDING = false;
atomic_bool STEPPER_SPEED_CONTROL_ENABLE = false;
atomic_uint_fast8_t STEPPER_STEP_STATE = 0; // 0-7
atomic_uint_fast16_t STEPPER_TICK_INTERVAL = 0xFFFF;
atomic_int_fast16_t STEPPER_POSITION_CONTROL_COUNT = 0;

/* Encoder */
atomic_int_fast16_t ENCODER_COUNT = 0;

/* PID */
const uint16_t PID_INTERVAL_200HZ = 40000;
atomic_bool DC_PID_ENABLE = false;
atomic_int_fast16_t PID_TARGET = 0;
int16_t ERROR_SUM = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    /* Configure Clocks */
    CSCTL0 = CSKEY;                                         // Write password to modify CS registers
    CSCTL1 = DCOFSEL0 + DCOFSEL1;                           // DCO = 8 MHz
    CSCTL2 = SELM0 + SELM1 + SELA0 + SELA1 + SELS0 + SELS1; // MCLK = DCO, ACLK = DCO, SMCLK = DCO
    CSCTL3 = DIVA_0 | DIVS_0 | DIVM_0;                      // MCLK = DCO/1, ACLK = DCO/1, SMCLK = DCO/1

    /* Configure Ports for UART A1 */
    P2SEL0 &= ~(BIT5 + BIT6);
    P2SEL1 |= BIT5 + BIT6;
    UCA1CTLW0 |= UCSWRST;                 // Put the UART in software reset
    UCA1CTLW0 |= UCSSEL0;                 // Run the UART using ACLK
    UCA1MCTLW = UCOS16 + UCBRF0 + 0x4900; // Baud rate = 9600 from an 8 MHz clock
    UCA1BRW = 52;
    UCA1CTLW0 &= ~UCSWRST; // release UART for operation
    UCA1IE |= UCRXIE;      // Enable UART Rx interrupt

    /* DEBUG: PJ Port */
    PJDIR |= BIT0;
    PJOUT &= ~BIT0;

    /* ---- DC Motor Control Start ---- */
    P3DIR |= BIT6 | BIT7; // P3.6: AIN2, P3.7: AIN1
    P3OUT &= ~(BIT6 | BIT7);

    /* Port 2: DC Motor Pins */
    P2DIR |= BIT1; // P2.1 as TB2.1 PWM output
    P2SEL0 |= BIT1;
    P2SEL1 &= ~BIT1;
    //
    // Timer B2.1 122Hz
    TB2CCTL1 = OUTMOD_7;
    TB2CCR1 = 0xFFFF >> 2;            // Duty Cycle
    TB2CTL = TBSSEL_2 | MC_2 | TBCLR; // SMCLK, Continuous mode, Clear
    /* ---- DC Motor Control End ---- */

    /* ---- Stepper Motor Control Start ---- */
    // Stepper Phase Look-Up Table //
    const bool phase_a1_sequence[8] = {1, 1, 0, 0, 0, 0, 0, 1}; // P1.5 TB0.2
    const bool phase_a2_sequence[8] = {0, 0, 0, 1, 1, 1, 0, 0}; // P1.4 TB0.1
    const bool phase_b1_sequence[8] = {0, 1, 1, 1, 0, 0, 0, 0}; // P3.5 TB1.2
    const bool phase_b2_sequence[8] = {0, 0, 0, 0, 0, 1, 1, 1}; // P3.4 TB1.1

    // Stepper Motor PWM Pins
    P1DIR |= BIT4 | BIT5; // AIN1: P1.5 TB0.2, AIN2: P1.4 TB0.1
    P1SEL0 |= BIT4 | BIT5;
    P1SEL1 &= ~(BIT4 | BIT5);

    P3DIR |= BIT4 | BIT5; // BIN1: P3.5 TB1.2, BIN2: P3.4 TB1.1
    P3SEL0 |= BIT4 | BIT5;
    P3SEL1 &= ~(BIT4 | BIT5);

    // Stepper Motor PWM Timers

    // Timer B0.1 B0.2 10kHz
    TB0CCTL1 = OUTMOD_7; // TB0.1
    TB0CCTL2 = OUTMOD_7; // TB0.2
    TB0CCR0 = 100;
    TB0CCR1 = 0;                      // Duty Cycle AIN2
    TB0CCR2 = 0;                      // Duty Cycle AIN1
    TB0CTL = TBSSEL_2 | MC_1 | TBCLR; // SMCLK, Up mode, Clear

    // Timer B1.1 B1.2 10kHz
    TB1CCTL1 = OUTMOD_7; // TB1.1
    TB1CCTL2 = OUTMOD_7; // TB1.2
    TB1CCR0 = 100;
    TB1CCR1 = 0;                      // Duty Cycle BIN2
    TB1CCR2 = 0;                      // Duty Cycle BIN1
    TB1CTL = TBSSEL_2 | MC_1 | TBCLR; // SMCLK, Up mode, Clear

    // Timer B2.2 Interrupt for Stepper Speed Control
    TB2CCTL2 |= CCIE;
    TB2CCR2 = 0x8000; // Software Timer Reload Value
    /* ---- Stepper Motor Control End ---- */

    /* ---- Encoder Timer A Initialization Start ---- */
    P1DIR &= ~BIT1; // P1.1 TA0.2 Up Encoder
    P1SEL0 |= BIT1;
    P1SEL1 &= ~BIT1;

    TA0CCTL2 = CM0 | SCS | CAP | CCIE;       // Rising Edge Only
    TA0CTL = TASSEL_2 | MC_2 | TACLR | TAIE; // SMCLK, Continuous mode, Clear, Enable Interrupt

    P1DIR &= ~BIT2; // P1.2 TA1.1 Down Encoder
    P1SEL0 |= BIT2;
    P1SEL1 &= ~BIT2;

    TA1CCTL1 = CM0 | SCS | CAP | CCIE;       // Rising Edge Only
    TA1CTL = TASSEL_2 | MC_2 | TACLR | TAIE; // SMCLK, Continuous mode, Clear, Enable Interrupt
    /* ---- Encoder Timer A Initialization End ---- */

    /* Debug: Timer A1.2 */
    P1DIR |= BIT3;
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT3;

    TA1CCTL2 = OUTMOD_7;
    TA1CCR2 = 0x4000;                 // Duty Cycle
    TA1CTL = TASSEL_2 | MC_2 | TACLR; // SMCLK, Continuous Mode, Clear

    /* ---- PID Control Timer A Initialization Start ---- */
    // TA0.1 200Hz
    TA0CCTL1 = CCIE;
    TA0CCR1 = PID_INTERVAL_200HZ;
    TA0CTL = TASSEL_2 | MC_2 | TACLR | TAIE; // SMCLK, Continuous Mode, Clear, Enable Interrupt
    /* ---- PID Control Timer A Initialization End ---- */

    __delay_cycles(4000000);

    // Enable Global Interrupt
    __enable_interrupt();

    while (1)
    {
        // Execute Stepper Motor State
        bool _true = true;
        if (atomic_compare_exchange_strong(&STEPPER_EXECUTION_PENDING, &_true, false))
        {
            uint8_t state = atomic_load(&STEPPER_STEP_STATE) % 8;
            TB0CCR2 = phase_a1_sequence[state] ? 25 : 0;
            TB0CCR1 = phase_a2_sequence[state] ? 25 : 0;
            TB1CCR2 = phase_b1_sequence[state] ? 25 : 0;
            TB1CCR1 = phase_b2_sequence[state] ? 25 : 0;
        }

        /* ----Packet Processing Start---- */
        Packet packet;
        if (!buffer_peek(&UART_RX_BUFFER, (uint8_t *)&packet, sizeof(Packet)))
        {
            // Not enough bytes
        }
        else if (!is_packet_valid(&packet))
        {
            // Invalid packet
            buffer_skip(&UART_RX_BUFFER, 1);
        }
        else // Valid Packet Received
        {
            buffer_skip(&UART_RX_BUFFER, sizeof(Packet));

            /* Echo Mode */
            if (packet.control_byte_0 == 0x00 && packet.control_byte_1 == 0x00)
            {
                for (uint8_t *p = &(packet.data_byte_0); p < &(packet.check_sum); ++p)
                {
                    while (!(UCA1IFG & UCTXIFG))
                        ;
                    UCA1TXBUF = *p;
                }
            }

            /* DC Motor Open-Loop Voltage */
            else if (packet.control_byte_0 == 0x01 && packet.control_byte_1 == 0x00)
            {
                atomic_store(&DC_PID_ENABLE, false);
                bool dir = packet.data_byte_0 & 0x01;
                TB2CCR1 = (packet.data_byte_1 << 8) + packet.data_byte_2;
                if (dir) // AIN1=high, AIN2=low
                {
                    P3OUT |= BIT7;  // AIN1 high
                    P3OUT &= ~BIT6; // AIN2 low
                }
                else
                {
                    P3OUT &= ~BIT7; // AIN1 low
                    P3OUT |= BIT6;  // AIN2 high
                }
            }

            /* Stepper Motor Single Half-Step */
            else if (packet.control_byte_0 == 0x02 && packet.control_byte_1 == 0x00)
            {
                // Disable Open-Loop Speed Control
                atomic_store(&STEPPER_SPEED_CONTROL_ENABLE, false);

                // Change Stepper Motor Global States
                atomic_store(&STEPPER_DIR, packet.data_byte_0 & 0x01);
                atomic_fetch_add(&STEPPER_STEP_STATE, atomic_load(&STEPPER_DIR) ? 1 : -1);

                // Request to Execute Stepper Motor State
                atomic_store(&STEPPER_EXECUTION_PENDING, true);
            }

            /* Stepper Motor Open-Loop Speed */
            else if (packet.control_byte_0 == 0x02 && packet.control_byte_1 == 0x01)
            {
                atomic_store(&STEPPER_DIR, packet.data_byte_0 & 0x01);
                atomic_store(&STEPPER_TICK_INTERVAL, (packet.data_byte_1 << 8) + packet.data_byte_2);
                atomic_store(&STEPPER_SPEED_CONTROL_ENABLE, true);
            }

            /* DC Motor Absolute Position */
            else if (packet.control_byte_0 == 0x01 && packet.control_byte_1 == 0x01)
            {
                atomic_store(&PID_TARGET, *(int16_t *)&packet.data_byte_1);
                atomic_store(&DC_PID_ENABLE, true);
            }

            /* DC Motor Relative Position */
            else if (packet.control_byte_0 == 0x01 && packet.control_byte_1 == 0x02)
            {
                atomic_fetch_add(&PID_TARGET, *(int16_t *)&packet.data_byte_1);
                atomic_store(&DC_PID_ENABLE, true);
            }

            /* Two Axis Position Control */
            else if (packet.control_byte_0 == 0x03 && packet.control_byte_1 == 0x00)
            {
                atomic_store(&STEPPER_SPEED_CONTROL_ENABLE, false);

                // Stepper Motor Relative Position
                atomic_fetch_add(&STEPPER_POSITION_CONTROL_COUNT, *(int16_t *)&packet.data_byte_5);

                // Stepper Motor Speed
                atomic_store(&STEPPER_TICK_INTERVAL, *(uint16_t *)&packet.data_byte_3);

                // DC Motor Position PID Target
                atomic_fetch_add(&PID_TARGET, *(int16_t *)&packet.data_byte_1);
                atomic_store(&DC_PID_ENABLE, true);
            }
        }
        /* ----Packet Processing End---- */
    } // end while(1)
} // end main()

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    uint8_t RxByte = UCA1RXBUF; // Get the new byte from the Rx buffer
    buffer_enqueue(&UART_RX_BUFFER, RxByte);
}

#pragma vector = TIMER2_B1_VECTOR
__interrupt void Timer_B2_ISR(void)
{
    switch (__even_in_range(TB2IV, TB2IV_TBIFG))
    {
    case TB2IV_TBCCR2: // Capture/compare 2 interrupt
        /* ---- Stepper Open Loop Speed Control ---- */
        if (atomic_load(&STEPPER_SPEED_CONTROL_ENABLE))
        {
            atomic_fetch_add(&STEPPER_STEP_STATE, atomic_load(&STEPPER_DIR) ? 1 : -1);
            atomic_store(&STEPPER_EXECUTION_PENDING, true);
            TB2CCR2 += atomic_load(&STEPPER_TICK_INTERVAL);
        }

        /* ---- Stepper Position Control ---- */
        else
        {
            int16_t count = atomic_load(&STEPPER_POSITION_CONTROL_COUNT);
            if (count != 0)
            {
                atomic_fetch_add(&STEPPER_STEP_STATE, count > 0 ? 1 : -1);
                atomic_store(&STEPPER_EXECUTION_PENDING, true);
                atomic_fetch_add(&STEPPER_POSITION_CONTROL_COUNT, count > 0 ? -1 : 1);
                TB2CCR2 += atomic_load(&STEPPER_TICK_INTERVAL);
            }
        }

        break;
    default:
        break;
    }
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG))
    {
    case TA0IV_TACCR2: // Encoder Up Count
        atomic_fetch_add(&ENCODER_COUNT, 1);
        TA1CCR2 += 1;
        TA0CCTL2 &= ~COV;
        break;
    case TA0IV_TACCR1: // PID Control Loop
        if (atomic_load(&DC_PID_ENABLE))
        {
            int16_t error = atomic_load(&PID_TARGET) - atomic_load(&ENCODER_COUNT);
            ERROR_SUM += error;

            int16_t kp_output;
            MPY32CTL0 = MPYM0;
            MPYS = error;
            OP2 = 275; // Kp
            __delay_cycles(3);
            kp_output = RESLO;

            int16_t ki_output;
            MPYS = ERROR_SUM;
            OP2 = 5; // Ki
            __delay_cycles(3);
            ki_output = RESLO;

            TB2CCR1 = abs(error) < 3 ? 0 : abs(kp_output + ki_output) + 9000;
            if (error < 0)
            {
                P3OUT |= BIT7;  // AIN1 high
                P3OUT &= ~BIT6; // AIN2 low
            }
            else
            {
                P3OUT &= ~BIT7; // AIN1 low
                P3OUT |= BIT6;  // AIN2 high
            }
            TA0CCR1 += PID_INTERVAL_200HZ;
            TA0CCTL1 &= ~COV;
        }
        break;
    default:
        break;
    }
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG))
    {
    case TA1IV_TACCR1: // Encoder Down Count
        atomic_fetch_add(&ENCODER_COUNT, -1);
        TA1CCR2 -= 1;
        TA1CCTL1 &= ~COV;
        break;
    default:
        break;
    }
}
