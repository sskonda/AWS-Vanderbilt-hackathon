
#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "final_threads.h"
#include "GFX_Library.h"
#include "vanderbilt_threads.h"
#include "general_drivers.h"

uint8_t last_state_sub_1 = 1;
uint8_t current_state_sub_1 = 0;

uint8_t last_state_sub_2 = 1;
uint8_t current_state_sub_2 = 0;

void Idle_Thread(void){

    while(1){}
}

char* sub_0 = "SUB 0";
char* sub_1 = "SUB 1";
char* sub_2 = "SUB 2";

void Draw_Subs()
{

    // Draw 2 subs

    G8RTOS_WaitSemaphore(&sem_SPI);

    for (int i = 0; i < 5; i++)
    {
        display_setCursor(30 + i*11, 320 - 30);
        display_setTextColor(0xFFFF);
        display_setTextSize(2);
        display_print(sub_1[i]);
    }

    for (int i = 0; i < 5; i++)
    {
        display_setCursor(150 + i*11, 320 - 30);
        display_setTextColor(0xFFFF);
        display_setTextSize(2);
        display_print(sub_2[i]);
    }

    G8RTOS_SignalSemaphore(&sem_SPI);

    while(1)
    {
        if (last_state_sub_2 != current_state_sub_2)
        {
            if (current_state_sub_2 == 0)
            {
                // Add semaphore here?
                for (int i = 0 + WIDTH; i < 2*WIDTH; i++)
                {
                    // Add semaphore here?
                    G8RTOS_WaitSemaphore(&sem_SPI);
                    for (int j = 0 + HEIGHT; j < 2*HEIGHT; j++)
                    {
                        ST7789_DrawPixel(i, j - 50, get_submarine_pixel(i - WIDTH, 2*HEIGHT - j));
                    }
                    G8RTOS_SignalSemaphore(&sem_SPI);
                }
            }
            else
            {
                for (int i = 0 + WIDTH; i < 2*WIDTH; i++)
                {
                    G8RTOS_WaitSemaphore(&sem_SPI);
                    for (int j = 0 + HEIGHT; j < 2*HEIGHT; j++)
                    {
                        ST7789_DrawPixel(i, j - 50, get_submarine_pixel_with_x(i - WIDTH, 2*HEIGHT - j));
                    }
                    G8RTOS_SignalSemaphore(&sem_SPI);
                }
            }
            last_state_sub_2 = current_state_sub_2;
        }

        if (last_state_sub_1 != current_state_sub_1)
        {
            if (current_state_sub_1 == 0)
            {
                // Add semaphore here?
                for (int i = 0; i < WIDTH; i++)
                {
                    // Add semaphore here?
                    G8RTOS_WaitSemaphore(&sem_SPI);
                    for (int j = 0 + HEIGHT; j < 2*HEIGHT; j++)
                    {
                        ST7789_DrawPixel(i, j - 50, get_submarine_pixel(i, 2*HEIGHT - j));
                    }
                    G8RTOS_SignalSemaphore(&sem_SPI);
                }
            }
            else
            {
                for (int i = 0; i < WIDTH; i++)
                {
                    G8RTOS_WaitSemaphore(&sem_SPI);
                    for (int j = 0 + HEIGHT; j < 2*HEIGHT; j++)
                    {
                        ST7789_DrawPixel(i, j - 50, get_submarine_pixel_with_x(i, 2*HEIGHT - j));
                    }
                    G8RTOS_SignalSemaphore(&sem_SPI);
                }
            }
            last_state_sub_1 = current_state_sub_1;
        }
        sleep(5);
    }
}

void Draw_Depth()
{
    // get depth
    uint32_t depth = 10;

    while (1)
    {
        if (depth == 10)
        {
            depth = 20;
        }
        else if (depth == 20)
        {
            depth = 30;
        }
        else if (depth == 30)
        {
            depth = 10;
        }

        // print depth
        char buffer[20] = {0};

        snprintf(buffer, 20, "Depth: %d", depth);

        int count = 0;
        char c = buffer[0];
        G8RTOS_WaitSemaphore(&sem_SPI);
        // black box

        ST7789_DrawRectangle(30, 45, 150, 20, 0);


        // data
        while(c != 0)
        {
            display_setCursor(30 + count*11, 60);
            display_setTextColor(0xFFFF);
            display_setTextSize(2);
            display_print(c);
            count += 1;
            c = buffer[count];
        }
        G8RTOS_SignalSemaphore(&sem_SPI);

        sleep(1000);
    }
}

void Read_ESP32()
{
    int c;

    while (1) {
        c = UART_ESP32_ReadChar();
        if (c != -1) {
            if (c == '1')
            {
                current_state_sub_1 = 0;
            }
            else if (c == '2')
            {
                current_state_sub_2 = 0;
            }
            else if (c == (int)'B')
            {
                current_state_sub_1 = 1;
            }
            else if (c == (int)'C')
            {
                current_state_sub_2 = 1;
            }
            UARTprintf("%c", (char)c);
        }
        sleep(10);
    }
}

void Read_Buttons() {
    // Initialize / declare any variables here
    uint8_t buttonVal;

    while(1)
    {
        G8RTOS_WaitSemaphore(&sem_GPIOE);
        // Sleep to debounce
        sleep(20);
        // Read the buttons status on the Multimod board.
        buttonVal = MultimodButtons_Get();

        if (buttonVal == 251){


            UARTprintf("Button 1 Pressed\n");

        }
        else if(buttonVal == 253){


            UARTprintf("Button 2 Pressed\n");

        }
        else if(buttonVal == 247){

            UARTprintf("Button 3 Pressed\n");
        }
        else if(buttonVal == 239){
            UARTprintf("Button 4 Pressed\n");
        }
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);
    }
}

void GPIOE_Handler() {
    // Disable interrupt
    // Signal relevant semaphore
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);
    G8RTOS_SignalSemaphore(&sem_GPIOE);
}

void UART4_Handler() {

    // Prepare to read data
    uint32_t status;
    uint8_t data[8];
    uint8_t i = 0;

    // Get interrupt status
    status = UARTIntStatus(UART4_BASE, true);

    // read through the FIFO (the entire frame of data)
    while(UARTCharsAvail(UART4_BASE)){

        // Store current data value
        data[i] = UARTCharGetNonBlocking(UART4_BASE);
        i++;
        //UARTCharPutNonBlocking(UART4_BASE, data);
    }
    i = 0;
    // Signal data ready
    UARTprintf("Data Ready :)");
    G8RTOS_WriteFIFO(0, (data[0] << 24) | data[1] << 16 | data[2] << 8 | data[3]);
    G8RTOS_WriteFIFO(0, (data[4] << 24) | data[5] << 16 | data[6] << 8 | data[7]);

    // Clear the asserted interrupts
    UARTIntClear(UART4_BASE, status);

}
