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
#define bSize   200
#define packet  5
#define time    6
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

struct Buffer
{
unsigned char rxBuffer[bSize];
char comparePacket[packet];
char Time[time];
};

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 *
 */
Void echoFxn(UArg arg0, UArg arg1)
{
    struct Buffer p;
    bool done = false;
    bool comp = false;
    //char ret;
    char temp;
    int i = 0;
    //UART_CONFIG_WLEN_MASK = 0x08;
   // unsigned char p.rxBuffer[200];
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
       UART_read(handle, p.rxBuffer, 200);                                      //Read in data from UART into rxBuffer (100 byte buffer)
       for(i = 0; i < 200; i++)                                                 //Iterate through buffer with for loop
       {
           if (p.rxBuffer[i] == '$')                                            //Start printing at the beginning of packet '$'
           {
               while(done == false)                                             //This while loop will iterate the buffer until a newline character is found
               {
                   comp = comparePackets(&p.rxBuffer, i);                       //Compares packets with "GPRMC", if true, then print to LCD
                   if (comp == true)
                   {
                       Time(&p.rxBuffer, i);                                    //This function creates an array with data for TIME
                       Display_print5(hDisplayLcd, 1, 2, "Packet: %c%c%c%c%c",  //This function prints the packet type
                       p.comparePacket[0], p.comparePacket[1],
                       p.comparePacket[2], p.comparePacket[3],
                       p.comparePacket[4]);

                       Display_print2(hDisplayLcd, 3, 2, "HR: %c%c", p.Time[0], p.Time[1]); //This function prints the Hour
                       Display_print2(hDisplayLcd, 4, 2, "MIN:%c%c", p.Time[2], p.Time[3]); //This function prints the Minutes
                       Display_print2(hDisplayLcd, 5, 2, "SEC:%c%c", p.Time[4], p.Time[5]); //This function prints the Seconds
                       Task_sleep(1000 * (1000/Clock_tickPeriod));              //Delay (1000 * (1000 / 48,000,000))
                   }
                   if (p.rxBuffer[i] == 10)                                     //If newline character is found, break loop and read in new data
                   {
                    done = true;                                                //Break while loop by setting done = to true
                    comp = false;                                               //Reset comp variable
                    i = 200;                                                    //Break for loop
                   }
                i++;                                                            //Increment i to iterate through buffer
               }
           }
        done = false;                                                           //Reset done variable to false
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

bool comparePackets(struct Buffer * b, int k)
{
   char cmp[] = {'G','P','R','M','C'};
   int i = 0;
   if (b->rxBuffer[k] == '$')
   {
       for(i = 0; i < 5; i++)
       {
           k++;
           b->comparePacket[i] = b->rxBuffer[k];
       }
   }
   int y;
   y = strncmp(cmp, b->comparePacket, 5);
   if (y == 0)
   {
      return true;
   }
   return false;
}

void Time(struct Buffer * b, int k)
{
int i = 0;
k += 7;
while(b->comparePacket[k] != ',' && i != 7)
{
    b->Time[i] = b->rxBuffer[k];
    i++;
    k++;
}
}

