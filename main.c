/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS SAR ADC basic example for
* ModusToolbox.
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
/* The status mask for the potentiometer SAR channel */
#define CYBSP_POT_CHAN_MSK          (1UL << CYBSP_POT_CHAN_IDX)

/* 8MHz IMO clock with 8000000 reload value to generate 1s interrupt */
#define SYSTICK_RELOAD_VAL          (8000000UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* HPPASS block ready status*/
volatile bool hppass_is_ready = false;

/* ADC conversion starting flag */
volatile bool start_adc_conversion = false;

/* SysTick one second flag */
volatile bool systick_one_second_flag = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* HPPASS interrupt handler */
void hppass_intr_handler(void);

/* SysTick interrupt handler */
void systick_intr_handler(void);

/* ADC channel result reading */
int16_t read_adc_channel_result(void);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
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

    /* Initialize the systick, set the 8MHz IMO as clock source */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, SYSTICK_RELOAD_VAL);
    /* Set Systick interrupt callback */
    Cy_SysTick_SetCallback(0, systick_intr_handler);

    /* The HPPASS interrupt configuration structure */
    cy_stc_sysint_t hppass_intr_config =
    {
        .intrSrc = pass_interrupt_mcpass_IRQn,
        .intrPriority = 0U,
    };
    /* Configure HPPASS interrupt */
    Cy_HPPASS_SetInterruptMask(CY_HPPASS_INTR_AC_INT);
    Cy_SysInt_Init(&hppass_intr_config, hppass_intr_handler);
    NVIC_EnableIRQ(hppass_intr_config.intrSrc);

    /* Start the HPPASS autonomous controller (AC) from state 0, didn't wait for HPPASS block ready */
    hppass_is_ready = false;
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, 0U))
    {
        CY_ASSERT(0);
    }

    /* CLear ADC conversion starting flag */
    start_adc_conversion = false;

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: SAR ADC basic example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to start or stop SAR ADC conversion\r\n");

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed() && (hppass_is_ready))
        {
            if(!start_adc_conversion)
            {
                start_adc_conversion = true;
                /* The potentiometer connected to SAR MUX input, and no sample time setting
                 * for group 0, discard first ADC conversion result.
                 * Note:if the global sample time enabled for group 0, no need this line, but it 
                 * will add more sampling times for each trigger.
                 */
                read_adc_channel_result();
                /* Enable Systick and the Systick interrupt */
                Cy_SysTick_Enable();
                systick_one_second_flag = false;
                printf("\r\nStart SAR ADC conversion\r\n");
            }
            else
            {
                start_adc_conversion = false;
                printf("Stop SAR ADC conversion\r\n");
                /* Disable Systick and the Systick interrupt */
                Cy_SysTick_Disable();
            }
        }

        /* If flag is true, trigger SAR ADC every second */
        if(start_adc_conversion && systick_one_second_flag)
        {
            systick_one_second_flag = false;
            /* Check SAR ADC busy status */
            if(!Cy_HPPASS_SAR_IsBusy())
            {
                /* Read ADC channel result */
                int16_t adc_result = read_adc_channel_result();
                /* Convert the result to voltage and print out the result */
                float32_t volts = Cy_HPPASS_SAR_CountsTo_Volts(CYBSP_POT_CHAN_IDX, 3300, adc_result); 
                printf("ADC channel result of potentiometer = 0x%x, voltage = %.4fV\r\n", adc_result, volts);
            }
        }
    }
}

/*******************************************************************************
* Function Name: hppass_intr_handler
********************************************************************************
* Summary:
* This function is the HPPASS interrupt handler (AC, etc.). Defined two states of
* HPPASS autonomous controller (AC):
* State 0 - Enable SAR and wait for block ready.
* State 1 - Set AC interrupt and stop AC.
* When AC perform to state 1, then send the AC interrupt to CPU to notify HPPASS
* block is ready.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void hppass_intr_handler(void)
{
    uint32_t intrStatus = Cy_HPPASS_GetInterruptStatusMasked();
    /* Clear interrupt */
    Cy_HPPASS_ClearInterrupt(intrStatus);

    /* Check AC interrupt */
    if(CY_HPPASS_INTR_AC_INT == (intrStatus & CY_HPPASS_INTR_AC_INT))
    {
        hppass_is_ready = true;
    }
}

/*******************************************************************************
* Function Name: read_adc_channel_result
********************************************************************************
* Summary:
* This function is the HPPASS SAR ADC channel result reading by polling.
*
* Parameters:
*  void
*
* Return:
*  int16_t - Return ADC conversion result.
*
*******************************************************************************/
int16_t read_adc_channel_result(void)
{
    uint32_t result_status = 0;
    int16_t channel_result = 0;

    /* Trigger SAR ADC group 0 conversion */
    Cy_HPPASS_SetFwTrigger(CY_HPPASS_TRIG_0_MSK);

    /* Wait for channel conversion done */
    do
    {
        result_status = Cy_HPPASS_SAR_Result_GetStatus();
    } while(!(result_status & CYBSP_POT_CHAN_MSK));

    /* Get channel data */
    channel_result = Cy_HPPASS_SAR_Result_ChannelRead(CYBSP_POT_CHAN_IDX);

    /* Clear result status */
    Cy_HPPASS_SAR_Result_ClearStatus(CYBSP_POT_CHAN_MSK);

    return channel_result;
}

/*******************************************************************************
* Function Name: systick_intr_handler
********************************************************************************
*
*  Summary:
*  Systick interrupt handler
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void systick_intr_handler(void)
{
    systick_one_second_flag = true;
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
