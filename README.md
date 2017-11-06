# seniorProj
Senior Project ECE 4800
/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>


/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>

#define TASKSTACKSIZE     768

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void echoFxn(UArg arg0, UArg arg1)
{
 //   char Packet[4] = {'GPRMC'};
    bool done = false;;
//    char ret;
    int i = 0;
    char temp;
    //UART_CONFIG_WLEN_MASK = 0x08;
    unsigned char rxBuffer[200];
    //int ret;
    UART_Handle handle;
    UART_Params uartParams;
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    Display_Handle hDisplayLcd = Display_open(Display_Type_LCD, &params);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;      //Not used, but Data is not processed mode
    uartParams.readDataMode = UART_DATA_BINARY;       //Data is not processed mode
    uartParams.readReturnMode = UART_RETURN_FULL;     //Receive return mode / UART_RETURN_FULL unblock/callback when buffer is full
    uartParams.readEcho = UART_ECHO_OFF;              //Echo received data back (false)
    uartParams.baudRate = 9600;                       //Set the baud rate of the GPS
    handle = UART_open(Board_UART0, &uartParams);     //

    if (handle == NULL) {
        System_abort("Error opening the UART");
    }

   //UART_write(uart, echoPrompt, sizeof(echoPrompt));


     /*
      *  ======== Parse ========
      *  Simple function for parsing data
      */
    while (1)
    {
       UART_read(handle, rxBuffer, 200);                                     //Read in data from UART into rxBuffer (100 byte buffer)
       for(i = 0; i < 200; i++)                                              //Iterate through buffer with for loop
       {
           if (rxBuffer[i] == '$')                                           //Start printing at the beginning of packet '$'
           {
               while(done == false)                                          //This while loop will iterate the buffer until a newline character is found
               {
             //  ret = rxBuffer[i];                                          //Set ret to rxBuffer[i]
             //  printf("UART: %c\n", ret);                                  //Print UART value from buffer
               temp = (char)rxBuffer[i];                                     //cast rxBuffer[i] to char
                   if (hDisplayLcd) {
                       Display_print1(hDisplayLcd, 5, 3, "GPS: %c", temp);   //Display character at row 5, column
                       Task_sleep(1000 * (1000/Clock_tickPeriod));           //Delay (1000 * (1000 / 48,000,000))
                   }
                   if (rxBuffer[i] == 10)                                    //If newline character is found, break loop and read in new data
                   {
                    done = true;                                             //Break while loop by setting done = to true
                    i = 200;                                                 //Break for loop
                   }
                i++;                                                         //Increment i to iterate through buffer
               }
           }
        done = false;                                                        //Reset done variable to false
       }
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    PIN_Handle ledPinHandle;
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initUART();

    /* Construct BIOS objects */


    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)echoFxn, &taskParams, NULL);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

char parse()
{

}
