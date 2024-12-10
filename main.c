/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the trigger example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/* The max. number of user timer interrupt */
#define USER_TIMER_INT_MAX_NUM              (10u)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* ADC channel result buffer */
uint16_t adc_result_buf = 0;

/* ADC group 0 interrupt flag */
volatile bool adc_group0_int_flag = false;

/* User timer interrupt flag */
volatile bool user_timer_int_flag = false;

/* Variable for the interrupt counter for user timer */
uint32_t user_timer_int_cnt = 0;

/* The flag of user timer running  */
volatile bool user_timer_is_running = false;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* ADC group 0 done interrupt handler */
void adc_group0_done_intr_handler(void);

/* User PWM1 interrupt handler */
void user_pwm1_intr_handler(void);

/* User timer interrupt handler */
void user_timer_intr_handler(void);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It configures three TCPWM counters and a ADC group
* to demonstrate one-to-one and on-to-many trigger concept. Here, first TCPWM
* counter trigger two TCPWMs in one-to-many configuration and the same first
* TCWPM counter also trigger HPPASS SAR ADC group 0 in one-to-one configuration.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize PWM 1 using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(USER_PWM1_HW, USER_PWM1_NUM, &USER_PWM1_config))
    {
        CY_ASSERT(0);
    }
    /* The user PWM1 interrupt configuration structure */
    cy_stc_sysint_t user_pwm1_intr_config =
    {
        .intrSrc = USER_PWM1_IRQ,
        .intrPriority = 0U,
    };
    /* Configure user PWM1 interrupt */
    Cy_SysInt_Init(&user_pwm1_intr_config, user_pwm1_intr_handler);
    NVIC_EnableIRQ(user_pwm1_intr_config.intrSrc);
    /* Enable the initialized PWM1 */
    Cy_TCPWM_PWM_Enable(USER_PWM1_HW, USER_PWM1_NUM);

    /* Initialize PWM 2 using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(USER_PWM2_HW, USER_PWM2_NUM, &USER_PWM2_config))
    {
        CY_ASSERT(0);
    }

    /* Initialize timer using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(USER_TIMER_HW, USER_TIMER_NUM, &USER_TIMER_config))
    {
        CY_ASSERT(0);
    }
    /* The user timer interrupt configuration structure */
    cy_stc_sysint_t user_timer_intr_config =
    {
        .intrSrc = USER_TIMER_IRQ,
        .intrPriority = 0U,
    };
    /* Configure user timer interrupt */
    Cy_SysInt_Init(&user_timer_intr_config, user_timer_intr_handler);
    NVIC_EnableIRQ(user_timer_intr_config.intrSrc);

    /* The interrupt configuration structure of ADC group 0 done */
    cy_stc_sysint_t adc_group0_done_intr_config =
    {
        .intrSrc = pass_interrupt_sar_entry_done_0_IRQn,
        .intrPriority = 0U,
    };
    /* Configure ADC group 0 done interrupt */
    Cy_HPPASS_SAR_Result_SetInterruptMask(CY_HPPASS_INTR_SAR_RESULT_GROUP_0);
    Cy_SysInt_Init(&adc_group0_done_intr_config, adc_group0_done_intr_handler);
    NVIC_EnableIRQ(adc_group0_done_intr_config.intrSrc);

    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* Clear global variables */
    adc_group0_int_flag = false;
    user_timer_is_running = false;
    user_timer_int_flag = false;
    user_timer_int_cnt = 0;

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("One-to-one and on-to-many trigger example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to start PWM1, then PWM1 generate out trigger to trigger ADC\r\n"
            "group 0, PWM2 and timer\r\n");

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed() && (!user_timer_is_running))
        {
            user_timer_int_cnt = 0;
            user_timer_is_running = true;
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
            /* Enable the initialized PWM2 */
            Cy_TCPWM_PWM_Enable(USER_PWM2_HW, USER_PWM2_NUM);
            /* Enable the initialized TIMER */
            Cy_TCPWM_Counter_Enable(USER_TIMER_HW, USER_TIMER_NUM);
            /* Start PWM1 */
            printf("\r\nStart user PWM1\r\n");
            Cy_TCPWM_TriggerStart_Single(USER_PWM1_HW, USER_PWM1_NUM);
        }

        /* Check whether the ADC conversion is complete */
        if(adc_group0_int_flag)
        {
            adc_group0_int_flag = false;
            /* Convert the ADC result to voltage */
            float32_t volts = Cy_HPPASS_SAR_CountsTo_Volts(CYBSP_POT_CHAN_IDX, 3300, adc_result_buf); 
            printf("ADC AN_B4 channel result = 0x%x, voltage = %.4fV\r\n", adc_result_buf, volts);
        }

        /* Check the interrupt flag of user timer */
        if(user_timer_int_flag)
        {
            user_timer_int_flag = false;
            printf("User timer terminal count (TC) interrupt, toggle USER_LED\r\n");
            /* Toggle user LED */
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            /* Check the interrupt times of user timer */
            user_timer_int_cnt++;
            if(USER_TIMER_INT_MAX_NUM <= user_timer_int_cnt)
            {
                user_timer_is_running = false;
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
                /* Disable PWM2 and timer */
                printf("Disable user PWM2 and timer\r\n");
                Cy_TCPWM_PWM_Disable(USER_PWM2_HW, USER_PWM2_NUM);
                Cy_TCPWM_Counter_Disable(USER_TIMER_HW, USER_TIMER_NUM);
            }
        }
    }
}

/*******************************************************************************
* Function Name: adc_group0_done_intr_handler
********************************************************************************
* Summary:
* This function is the ADC group 0 done interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_group0_done_intr_handler(void)
{
    /* Clear SAR result interrupt */
    Cy_HPPASS_SAR_Result_ClearInterrupt(CY_HPPASS_INTR_SAR_RESULT_GROUP_0);
    /* Read channel result from register */
    adc_result_buf = Cy_HPPASS_SAR_Result_ChannelRead(CYBSP_POT_CHAN_IDX);
    adc_group0_int_flag = true;
}

/*******************************************************************************
* Function Name: user_pwm1_intr_handler
********************************************************************************
* Summary:
* This is the user PWM1 interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void user_pwm1_intr_handler(void)
{
    uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(USER_PWM1_HW, USER_PWM1_NUM);
    Cy_TCPWM_ClearInterrupt(USER_PWM1_HW, USER_PWM1_NUM, intrStatus);
    if(CY_TCPWM_INT_ON_TC == (intrStatus & CY_TCPWM_INT_ON_TC))
    {
        printf("User PWM1 terminal count (TC) interrupt\r\n");
    }
}

/*******************************************************************************
* Function Name: user_timer_intr_handler
********************************************************************************
* Summary:
* This is the user timer interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void user_timer_intr_handler(void)
{
    uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(USER_TIMER_HW, USER_TIMER_NUM);
    Cy_TCPWM_ClearInterrupt(USER_TIMER_HW, USER_TIMER_NUM, intrStatus);
    if(CY_TCPWM_INT_ON_TC == (intrStatus & CY_TCPWM_INT_ON_TC))
    {
        user_timer_int_flag = true;
    }
}

/*******************************************************************************
* Function Name: user_button_is_pressed
****************************************************************************//**
* Summary:
*  Check if the user button is pressed.
*
* Return:
*  Returns the status of user button.
*
*******************************************************************************/
bool user_button_is_pressed(void)
{
    uint32_t pressCount = 0;

    if(Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) != CYBSP_BTN_PRESSED)
    {
        return false;
    }
    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);
        pressCount++;
    }
    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(10);

    if(10 < pressCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* [] END OF FILE */
