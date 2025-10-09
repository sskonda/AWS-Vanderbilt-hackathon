
#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "final_threads.h"
#include "GFX_Library.h"
#include "vanderbilt_threads.h"
#include "general_drivers.h"
#include <math.h>

#define SUB_0

uint8_t last_state_sub_1 = 1;
uint8_t current_state_sub_1 = 0;

uint8_t last_state_sub_2 = 1;
uint8_t current_state_sub_2 = 0;

Tilt_t  tilt_data = {0};
ShipState ship_state = {0};

void Idle_Thread(void){

    while(1){}
}

void ST7789_DrawThickLine5(uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1,
                           uint16_t color)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;
    double len = sqrt((double)dx*dx + (double)dy*dy);

    // If the endpoints coincide, draw a 5x5 filled square centered at the point
    if (len < 1e-6) {
        int half = 2; // because width = 5 -> offsets -2..+2
        for (int yy = -half; yy <= half; ++yy) {
            for (int xx = -half; xx <= half; ++xx) {
                int px = (int)x0 + xx;
                int py = (int)y0 + yy;
                if (px < 0) px = 0;
                if (py < 0) py = 0;
                ST7789_DrawLine((uint16_t)px, (uint16_t)py, (uint16_t)px, (uint16_t)py, color);
            }
        }
        return;
    }

    // Perpendicular vector to the line is (-dy, dx).
    // Normalize it and draw five parallel lines offset by -2..+2 units.
    for (int offset = -2; offset <= 2; ++offset) {
        int shift_x = (int) round(( -dy * (double)offset ) / len);
        int shift_y = (int) round((  dx * (double)offset ) / len);

        int sx0 = (int)x0 + shift_x;
        int sy0 = (int)y0 + shift_y;
        int sx1 = (int)x1 + shift_x;
        int sy1 = (int)y1 + shift_y;

        // Prevent negative cast issues (optional clamp; adapt if you have known screen extents)
        if (sx0 < 0) sx0 = 0;
        if (sy0 < 0) sy0 = 0;
        if (sx1 < 0) sx1 = 0;
        if (sy1 < 0) sy1 = 0;

        ST7789_DrawLine((uint16_t)sx0, (uint16_t)sy0, (uint16_t)sx1, (uint16_t)sy1, color);
    }
}

void ST7789_Draw3DAxes_C(uint16_t x0, uint16_t y0)
{
    const float length = 100.0f;
    const float half = length * 0.5f;

    /* Colors in RGB565 */
    const uint16_t COLOR_X = 0xF800; // red
    const uint16_t COLOR_Y = 0x07E0; // green
    const uint16_t COLOR_Z = 0x001F; // blue

    int sx0, sy0, sx1, sy1;
    /* X axis: (-half, 0, 0) -> (+half, 0, 0) */
    project3D(-half, 0.0f, 0.0f, &sx0, &sy0);
    project3D( half, 0.0f, 0.0f, &sx1, &sy1);
    {
        int ex0 = (int)x0 + sx0;
        int ey0 = (int)y0 + sy0;
        int ex1 = (int)x0 + sx1;
        int ey1 = (int)y0 + sy1;
        if (ex0 < 0) ex0 = 0; if (ey0 < 0) ey0 = 0;
        if (ex1 < 0) ex1 = 0; if (ey1 < 0) ey1 = 0;
        ST7789_DrawLine((uint16_t)ex0, (uint16_t)ey0, (uint16_t)ex1, (uint16_t)ey1, COLOR_X);
    }

    /* Y axis: (0, -half, 0) -> (0, +half, 0) */
    int yx0, yy0, yx1, yy1;
    project3D(0.0f, -half, 0.0f, &yx0, &yy0);
    project3D(0.0f,  half, 0.0f, &yx1, &yy1);
    {
        int ex0 = (int)x0 + yx0;
        int ey0 = (int)y0 + yy0;
        int ex1 = (int)x0 + yx1;
        int ey1 = (int)y0 + yy1;
        if (ex0 < 0) ex0 = 0; if (ey0 < 0) ey0 = 0;
        if (ex1 < 0) ex1 = 0; if (ey1 < 0) ey1 = 0;
        ST7789_DrawLine((uint16_t)ex0, (uint16_t)ey0, (uint16_t)ex1, (uint16_t)ey1, COLOR_Y);
    }

    /* Z axis: (0,0,-half) -> (0,0,+half) */
    int zx0, zy0, zx1, zy1;
    project3D(0.0f, 0.0f, -half, &zx0, &zy0);
    project3D(0.0f, 0.0f,  half, &zx1, &zy1);
    {
        int ex0 = (int)x0 + zx0;
        int ey0 = (int)y0 + zy0;
        int ex1 = (int)x0 + zx1;
        int ey1 = (int)y0 + zy1;
        if (ex0 < 0) ex0 = 0; if (ey0 < 0) ey0 = 0;
        if (ex1 < 0) ex1 = 0; if (ey1 < 0) ey1 = 0;
        ST7789_DrawLine((uint16_t)ex0, (uint16_t)ey0, (uint16_t)ex1, (uint16_t)ey1, COLOR_Z);
    }

    /* small ticks/markers at the positive ends (optional) */
    const int tick = 6;
    /* X positive tick */
    {
        int ex = (int)x0 + sx1;
        int ey = (int)y0 + sy1;
        if (ex < 0) ex = 0; if (ey < 0) ey = 0;
        ST7789_DrawLine((uint16_t)(ex - tick), (uint16_t)(ey - tick/2),
                        (uint16_t)(ex + tick), (uint16_t)(ey + tick/2),
                        COLOR_X);
    }
    /* Y positive tick (top) */
    {
        int ex = (int)x0 + yx0;
        int ey = (int)y0 + yy0;
        if (ex < 0) ex = 0; if (ey < 0) ey = 0;
        ST7789_DrawLine((uint16_t)(ex - tick/2), (uint16_t)(ey + tick),
                        (uint16_t)(ex + tick/2), (uint16_t)(ey - tick),
                        COLOR_Y);
    }
    /* Z positive tick */
    {
        int ex = (int)x0 + zx1;
        int ey = (int)y0 + zy1;
        if (ex < 0) ex = 0; if (ey < 0) ey = 0;
        ST7789_DrawLine((uint16_t)(ex - tick), (uint16_t)(ey - tick),
                        (uint16_t)(ex + tick), (uint16_t)(ey + tick),
                        COLOR_Z);
    }
}

void ST7789_Draw3DVectorThick5(uint16_t x0, uint16_t y0,
                               float ux, float uy, float uz,
                               uint16_t color)
{
    const float draw_length = 50.0f; /* draw full 50 pixels from center */
    /* normalize the input vector (defensive even if caller says unit vector) */
    float vlen = sqrtf(ux*ux + uy*uy + uz*uz);
    if (vlen <= 1e-6f) {
        /* Degenerate vector: draw a 5x5 dot centered at x0,y0 */
        const int half = 2; /* 5 pixels -> offsets -2..+2 */
        for (int yy = -half; yy <= half; ++yy) {
            for (int xx = -half; xx <= half; ++xx) {
                int px = (int)x0 + xx;
                int py = (int)y0 + yy;
                if (px < 0) px = 0; if (py < 0) py = 0;
                ST7789_DrawLine((uint16_t)px, (uint16_t)py, (uint16_t)px, (uint16_t)py, color);
            }
        }
        return;
    }
    ux /= vlen; uy /= vlen; uz /= vlen;

    /* compute 3D endpoint = unit_vector * draw_length */
    float ex3 = ux * draw_length;
    float ey3 = uy * draw_length;
    float ez3 = uz * draw_length;

    /* project start (0,0,0) and end to 2D (relative offsets) */
    int srx, sry, erx, ery;
    project3D(0.0f, 0.0f, 0.0f, &srx, &sry);
    project3D(ex3, ey3, ez3, &erx, &ery);

    /* translate to actual screen coords (start should be exactly x0,y0) */
    int sx = (int)x0 + srx;
    int sy = (int)y0 + sry;
    int ex = (int)x0 + erx;
    int ey = (int)y0 + ery;

    /* If projected 2D line is essentially a point, draw small 5x5 dot */
    int dx = ex - sx;
    int dy = ey - sy;
    float len2d = sqrtf((float)dx*(float)dx + (float)dy*(float)dy);
    if (len2d <= 0.5f) {
        const int half = 2;
        for (int yy = -half; yy <= half; ++yy) {
            for (int xx = -half; xx <= half; ++xx) {
                int px = sx + xx;
                int py = sy + yy;
                if (px < 0) px = 0; if (py < 0) py = 0;
                ST7789_DrawLine((uint16_t)px, (uint16_t)py, (uint16_t)px, (uint16_t)py, color);
            }
        }
        return;
    }

    /* Draw 5 parallel 1-pixel lines offset along the perpendicular (-dy, dx).
       Offsets are -2..+2 to make total width = 5 pixels. */
    for (int offset = -2; offset <= 2; ++offset) {
        /* compute floating shift amount */
        float shift_fx = (- (float)dy * (float)offset) / len2d;
        float shift_fy = (  (float)dx * (float)offset) / len2d;
        int shift_x = fround_to_int(shift_fx);
        int shift_y = fround_to_int(shift_fy);

        int sx_o = sx + shift_x;
        int sy_o = sy + shift_y;
        int ex_o = ex + shift_x;
        int ey_o = ey + shift_y;

        /* clamp negatives to avoid underflow into uint when casting;
           if you know screen bounds, clamp to width/height too */
        if (sx_o < 0) sx_o = 0; if (sy_o < 0) sy_o = 0;
        if (ex_o < 0) ex_o = 0; if (ey_o < 0) ey_o = 0;

        ST7789_DrawLine((uint16_t)sx_o, (uint16_t)sy_o,
                        (uint16_t)ex_o, (uint16_t)ey_o,
                        color);
    }
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

void printFloatXX_UART4(float val) {
    char buffer[20];
    char* c;
    int sign = (val < 0.0f) ? -1 : 1;
    if (val < 0.0f) val = -val;  /* make positive for math */

    /* Scale and round to two decimals */
    int scaled = (int)(val * 100.0f + 0.5f);  /* e.g. 0.567 -> 57 */

    int integer_part = scaled / 100;   /* 0 */
    int decimal_part = scaled % 100;   /* 57 */

    if (sign < 0)
    {
        snprintf(buffer, 20, "-%d.%02d", integer_part, decimal_part);
        c = buffer;
        while (*c) {
            UARTCharPut(UART4_BASE, (unsigned char)(*c++));
        }
    }
    else
    {
        snprintf(buffer, 20, "%d.%02d", integer_part, decimal_part);
        c = buffer;
        while (*c) {
            UARTCharPut(UART4_BASE, (unsigned char)(*c++));
        }
    }
}

char* sub_0 = "SUB 0";
char* sub_1 = "SUB 1";
char* sub_2 = "SUB 2";

void Draw_Subs()
{

    // Draw 2 subs

    G8RTOS_WaitSemaphore(&sem_SPI);

#ifdef SUB_0
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
#endif

#ifdef SUB_1
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
        display_print(sub_2[i]);
    }
#endif

#ifdef SUB_2
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
#endif



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

void Draw_Position()
{
    // draw position text
    char* position_text = "Position";
    char buffer[10] = {0};
    char c;
    uint32_t count;

    G8RTOS_WaitSemaphore(&sem_SPI);

    for (int i = 0; i < 8; i++)
    {
        display_setCursor(125 + 11*i, 150);
        display_setTextColor(0xFFFF);
        display_setTextSize(2);
        display_print(position_text[i]);
    }

    G8RTOS_SignalSemaphore(&sem_SPI);

    while(1)
    {

        G8RTOS_WaitSemaphore(&sem_SPI);

        ST7789_DrawRectangle(125, 105, 90, 20, 0);

        // print x
        snprintf(buffer, 10, "X: %d", ship_state.px);
        c = buffer[0];
        count = 0;
        while (c != 0)
        {
            display_setCursor(125 + count*11, 120);
            display_setTextColor(0xFFFF);
            display_setTextSize(2);
            display_print(c);
            count += 1;
            c = buffer[count];
        }

        ST7789_DrawRectangle(125, 75, 90, 20, 0);

        // print y
        snprintf(buffer, 10, "Y: %d", ship_state.py);
        c = buffer[0];
        count = 0;
        while (c != 0)
        {
            display_setCursor(125 + count*11, 90);
            display_setTextColor(0xFFFF);
            display_setTextSize(2);
            display_print(c);
            count += 1;
            c = buffer[count];
        }

        ST7789_DrawRectangle(125, 45, 90, 20, 0);

        // print z
        snprintf(buffer, 10, "Z: %d", ship_state.pz);
        c = buffer[0];
        count = 0;
        while (c != 0)
        {
            display_setCursor(125 + count*11, 60);
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


void Draw_Data()
{

    uint16_t x0 = 50;
    uint16_t y0 = 100;
    float prev_ship_fx = ship_state.fx;
    float prev_ship_fy = ship_state.fy;
    float prev_ship_fz = ship_state.fz;

    ST7789_Draw3DAxes_C(x0, y0);

    while(1)
    {
        G8RTOS_WaitSemaphore(&sem_SPI);

        ST7789_Draw3DVectorThick5(x0, y0, prev_ship_fz, (-1.0)*prev_ship_fy, prev_ship_fx, 0x0000);

        ST7789_Draw3DVectorThick5(x0, y0, ship_state.fz, (-1.0)*ship_state.fy, ship_state.fx, 0xFFFF);

        prev_ship_fx = ship_state.fx;
        prev_ship_fy = ship_state.fy;
        prev_ship_fz = ship_state.fz;

        G8RTOS_SignalSemaphore(&sem_SPI);


        sleep(100);

        G8RTOS_WaitSemaphore(&sem_SPI);
        ST7789_Draw3DAxes_C(x0, y0);
        G8RTOS_SignalSemaphore(&sem_SPI);
        sleep(100);
    }
}

void Read_ESP32()
{
    int c;

    char* zero_f = "E0F\n";
    char* one_f = "E1F\n";
    char* two_f = "E2F\n";

    char* zero_l = "E0L\n";
    char* one_l = "E1L\n";
    char* two_l = "E2L\n";

    char* c_uart;

    while (1) {
        c = UART_ESP32_ReadChar();
        if (c != -1) {
            G8RTOS_WaitSemaphore(&sem_UART4);
#ifdef SUB_0
            if (c == '1')
            {
                current_state_sub_1 = 0;
                c_uart = one_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
            }
            else if (c == '2')
            {
                c_uart = two_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 0;
            }
            else if (c == (int)'B')
            {
                c_uart = one_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_1 = 1;
            }
            else if (c == (int)'C')
            {
                c_uart = two_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 1;
            }
#endif

#ifdef SUB_1
            if (c == '0')
            {
                c_uart = zero_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_1 = 0;
            }
            else if (c == '2')
            {
                c_uart = two_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 0;
            }
            else if (c == (int)'A')
            {
                c_uart = zero_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_1 = 1;
            }
            else if (c == (int)'C')
            {
                c_uart = two_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 1;
            }
#endif

#ifdef SUB_2
            if (c == '0')
            {
                c_uart = zero_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_1 = 0;
            }
            else if (c == '1')
            {
                c_uart = one_f;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 0;
            }
            else if (c == (int)'A')
            {
                c_uart = zero_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_1 = 1;
            }
            else if (c == (int)'B')
            {
                c_uart = one_l;
                while (*c_uart) {
                    UARTCharPut(UART4_BASE, (unsigned char)(*c_uart++));
                }
                current_state_sub_2 = 1;
            }
#endif

            G8RTOS_SignalSemaphore(&sem_UART4);
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

//            temp_char = s_1;
            UARTprintf("Button 1 Pressed\n");
//            while (*temp_char) {
//                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
//            }

        }
        else if(buttonVal == 253){

//            temp_char = s_2;
            UARTprintf("Button 2 Pressed\n");
//            while (*temp_char) {
//                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
//            }

        }
        else if(buttonVal == 247){

//            temp_char = s_3;
            UARTprintf("Button 3 Pressed\n");
//            while (*temp_char) {
//                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
//            }
        }
        else if(buttonVal == 239){

//            temp_char = s_4;
            UARTprintf("Button 4 Pressed\n");
//            while (*temp_char) {
//                UARTCharPut(UART4_BASE, (unsigned char)(*temp_char++));
//            }
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

void Send_PO_Data(void)
{
    char buffer[60] = {0};
    char* c;

    while(1)
    {
        G8RTOS_WaitSemaphore(&sem_UART4);
        // Position
        snprintf(buffer, 60, "P,%d,%d,%d\n", ship_state.px, ship_state.py, ship_state.pz);
        c = buffer;
        while (*c) {
            UARTCharPut(UART4_BASE, (unsigned char)(*c++));
        }
        sleep(10);
        // Orientation

        UARTCharPut(UART4_BASE, (unsigned char)'O');
        UARTCharPut(UART4_BASE, (unsigned char)',');
        printFloatXX_UART4(ship_state.fx);
        UARTCharPut(UART4_BASE, (unsigned char)',');
        printFloatXX_UART4(ship_state.fy);
        UARTCharPut(UART4_BASE, (unsigned char)',');
        printFloatXX_UART4(ship_state.fz);
        UARTCharPut(UART4_BASE, (unsigned char)'\n');

        G8RTOS_SignalSemaphore(&sem_UART4);

        sleep(500);
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
