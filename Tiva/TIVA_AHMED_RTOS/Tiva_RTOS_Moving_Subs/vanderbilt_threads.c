
#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "final_threads.h"
#include "GFX_Library.h"
#include "vanderbilt_threads.h"
#include "general_drivers.h"
#include <math.h>

uint8_t last_state_sub_1 = 1;
uint8_t current_state_sub_1 = 0;

uint8_t last_state_sub_2 = 1;
uint8_t current_state_sub_2 = 0;

Tilt_t  tilt_data = {0};
ShipState ship_state = {0};

void Idle_Thread(void){

    while(1){}
}

void computeTilt(int16_t ax, int16_t ay, int16_t az, Tilt_t *t) {
    if (t == NULL) return;

    /* Normalize to -1.0 to +1.0 */
    float x = ax / GRAVITY;
    float y = ay / GRAVITY;
    float z = az / GRAVITY;

    /* Compute tilt angles (radians) based on your axis orientation */
    float forward_angle = atan2f(y, sqrtf(x * x + z * z));  /* Y vs Z */
    float side_angle    = atan2f(x, sqrtf(y * y + z * z));  /* X vs Z */

    /* Normalize to range -1 to +1 (PI/2 corresponds to 1) */
    t->forward = (-1.0)*(forward_angle / (PI / 2.0f));
    t->side    = (-1.0)*(side_angle / (PI / 2.0f));
}

void printFloatXX(float val) {
    int sign = (val < 0.0f) ? -1 : 1;
    if (val < 0.0f) val = -val;  /* make positive for math */

    /* Scale and round to two decimals */
    int scaled = (int)(val * 100.0f + 0.5f);  /* e.g. 0.567 -> 57 */

    int integer_part = scaled / 100;   /* 0 */
    int decimal_part = scaled % 100;   /* 57 */

    if (sign < 0)
        UARTprintf("-%d.%02d", integer_part, decimal_part);
    else
        UARTprintf("%d.%02d", integer_part, decimal_part);
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
        display_print(sub_0[i]);
    }

    for (int i = 0; i < 5; i++)
    {
        display_setCursor(150 + i*11, 320 - 30);
        display_setTextColor(0xFFFF);
        display_setTextSize(2);
        display_print(sub_1[i]);
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
                        ST7789_DrawPixel(i, j + 110, get_submarine_pixel(i - WIDTH, 2*HEIGHT - j));
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
                        ST7789_DrawPixel(i, j + 110, get_submarine_pixel_with_x(i - WIDTH, 2*HEIGHT - j));
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
                        ST7789_DrawPixel(i, j + 110, get_submarine_pixel(i, 2*HEIGHT - j));
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
                        ST7789_DrawPixel(i, j + 110, get_submarine_pixel_with_x(i, 2*HEIGHT - j));
                    }
                    G8RTOS_SignalSemaphore(&sem_SPI);
                }
            }
            last_state_sub_1 = current_state_sub_1;
        }
        sleep(5);
    }
}

void Get_Data()
{
    int16_t accel_data[3] = {0};
    int32_t temp_joystick_data = 0;

    uint16_t joystickY = 0;
    float normalY = 0;
    ship_state.fx = 1.0;

    while(1)
    {
        // get accel data
        G8RTOS_WaitSemaphore(&sem_I2C);
        BMI160_AccelXYZGetResult(accel_data);
        G8RTOS_SignalSemaphore(&sem_I2C);

        // fill tilt_data
        computeTilt(accel_data[0], accel_data[1], accel_data[2], &tilt_data);

        G8RTOS_WaitSemaphore(&sem_UART);
        UARTprintf("Forward: ");
        printFloatXX(tilt_data.forward);
        UARTprintf("  Side: ");
        printFloatXX(tilt_data.side);
        G8RTOS_SignalSemaphore(&sem_UART);

        temp_joystick_data = G8RTOS_ReadFIFO(1);
        joystickY = temp_joystick_data;

        normalY = 2.0*(((float)(joystickY)) / 4096.0) - 1.0;

        if(normalY > 0.3){
            normalY = normalY;
        }
        else{
            normalY = 0;
        }

        ship_state.thrust = normalY * 10.0;
        updateShipPosition(&ship_state, tilt_data);

        G8RTOS_WaitSemaphore(&sem_UART);
        UARTprintf("Position x: %d y: %d z: %d", ship_state.px, ship_state.py, ship_state.pz);
        G8RTOS_SignalSemaphore(&sem_UART);


        G8RTOS_WaitSemaphore(&sem_UART);
        UARTprintf(" Direction: X: ");
        printFloatXX(ship_state.fx);
        UARTprintf("  Y: ");
        printFloatXX(ship_state.fy);
        UARTprintf(" Z: ");
        printFloatXX(ship_state.fz);
        UARTprintf("\n");
        G8RTOS_SignalSemaphore(&sem_UART);

        sleep(1000);
    }
}

void Move_Ship()
{

}

void Draw_Data()
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
            if (c == '0')
            {
                current_state_sub_1 = 0;
            }
            else if (c == '1')
            {
                current_state_sub_2 = 0;
            }
            else if (c == (int)'A')
            {
                current_state_sub_1 = 1;
            }
            else if (c == (int)'B')
            {
                current_state_sub_2 = 1;
            }
            G8RTOS_WaitSemaphore(&sem_UART);
            UARTprintf("%c", (char)c);
            G8RTOS_SignalSemaphore(&sem_UART);
        }
        sleep(10);
    }
}

void Read_Buttons() {
    // Initialize / declare any variables here
    uint8_t buttonVal;

    const char* s_1 = "wow_one\n";
    const char* s_2 = "wow_two\n";
    const char* s_3 = "wow_three\n";
    const char* s_4 = "wow_four\n";

    char* temp_char;

    while(1)
    {
        G8RTOS_WaitSemaphore(&sem_GPIOE);
        // Sleep to debounce
        sleep(20);
        // Read the buttons status on the Multimod board.
        buttonVal = MultimodButtons_Get();

        G8RTOS_WaitSemaphore(&sem_UART);
        if (buttonVal == 251){

            temp_char = s_1;
            UARTprintf("Button 1 Pressed\n");
            while (*temp_char) {
                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
            }

        }
        else if(buttonVal == 253){

            temp_char = s_2;
            UARTprintf("Button 2 Pressed\n");
            while (*temp_char) {
                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
            }

        }
        else if(buttonVal == 247){

            temp_char = s_3;
            UARTprintf("Button 3 Pressed\n");
            while (*temp_char) {
                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
            }
        }
        else if(buttonVal == 239){

            temp_char = s_4;
            UARTprintf("Button 4 Pressed\n");
            while (*temp_char) {
                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
            }
        }
        G8RTOS_SignalSemaphore(&sem_UART);
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);
    }
}

void BeagleBone_Do()
{
    uint32_t top4 = 0;
    uint32_t bot4 = 0;
    uint8_t data[8] = {0};

    while(1)
    {
        // get
        top4 = G8RTOS_ReadFIFO(0);
        bot4 = G8RTOS_ReadFIFO(0);

        // parse
        for (int i = 0; i < 4; i++)
        {
            data[i] = (top4 >> (8*(3-i))) & 0xFF;
        }

        for (int i = 4; i < 8; i++)
        {
            data[i] = (bot4 >> (8*(3-(i-4)))) & 0xFF;
        }

        // print data
        G8RTOS_WaitSemaphore(&sem_UART);
        for (int i = 0; i < 8; i++)
        {
            UARTprintf("Data %d: %d\n", i, data[i]);
        }
        G8RTOS_SignalSemaphore(&sem_UART);

        // do with data

        G8RTOS_WaitSemaphore(&sem_UART);
        if (data[0] == '1')
        {
            // change states or whatever
            UARTprintf("STATE 1\n");
        }
        else if (data[0] == '2')
        {
            // change states or whatever
            UARTprintf("STATE 2\n");
        }
        else if (data[0] == '3')
        {
            // change states or whatever
            UARTprintf("STATE 3\n");
        }
        G8RTOS_SignalSemaphore(&sem_UART);

        sleep(5);
    }
}

// Periodic Threads
void Get_Joystick(void) {
    // Read the joystick

    uint32_t temp = JOYSTICK_GetXY();
    // Send through FIFO.
    G8RTOS_WriteFIFO(1, temp);

}

// Handlers
void GPIOE_Handler() {
    // Disable interrupt
    // Signal relevant semaphore
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);
    G8RTOS_SignalSemaphore(&sem_GPIOE);
}

void UART4_Handler() {

    // Prepare to read data
    uint32_t status;
    uint8_t data[8] = {0};
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
    UARTprintf("Data Ready :)\n");
    G8RTOS_WriteFIFO(0, (data[0] << 24) | data[1] << 16 | data[2] << 8 | data[3]);
    G8RTOS_WriteFIFO(0, (data[4] << 24) | data[5] << 16 | data[6] << 8 | data[7]);

    // Clear the asserted interrupts
    UARTIntClear(UART4_BASE, status);

}
