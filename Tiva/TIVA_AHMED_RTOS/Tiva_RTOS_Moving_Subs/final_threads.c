
// Includes

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
#include "final_threads.h"
#include "GFX_Library.h"
#include <time.h>
#include <stdlib.h>

uint16_t TopRightX = 220;
uint16_t TopLeftX = 20;
uint16_t BottomRightX = 220;
uint16_t BottomLeftX = 20;
uint16_t TopRightY = 240;
uint16_t TopLeftY = 240;
uint16_t BottomRightY = 40;
uint16_t BottomLeftY = 40;

uint16_t top_color = ST7789_RED;
uint16_t bottom_color = ST7789_BLUE;
uint16_t right_color = ST7789_GREEN;
uint16_t left_color = ST7789_WHITE;

uint8_t state = 2;

uint16_t playerX = 40;
uint16_t playerY = 50;
int16_t playerV = 0;
int16_t playerA = -3;

uint16_t score = 0;

enemy_t enemy[3];

// Y - 50 to 230
// X - 30 to 210

uint8_t numEnemy = 1;

uint8_t render_border = 1;
uint8_t render_obstacles = 0;
uint8_t start_animation = 0;
uint8_t roofed = 0;
uint8_t joystick_flag = 1;

uint8_t obstacle_bm[19][19] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0},
                           {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                           {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

uint8_t obstacle_temp[19][19] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

char* run_char = "RUN";
char* name_char = "By Ahmed Sattar";
char* game_char = "Game Over";
char* score_char = "Score ";


void Idle_Thread(void){
    time_t t;
    srand((unsigned) time(&t));
    while(1){
        //UARTprintf("Working\n");
    }
}

void Score_Thread(void){

    uint16_t temp_score;
    uint16_t prev_score = 0;
    uint16_t digit;
    char temp_char;



    while(1){

        while(state != 0){}

        temp_score = prev_score;

        G8RTOS_WaitSemaphore(&sem_SPI);

        for(int i = 0; i < 6; i++){
            display_setCursor(69 + i*11, 270);
            display_setTextColor(0xFFFF);
            display_setTextSize(2);
            display_print(score_char[i]);
        }

        for (int i = 0; i < 5; i++){

            digit = temp_score % 10;
            temp_char = digit + '0';
            display_setCursor(180 - i*11, 270);
            display_setTextColor(0);
            display_setTextSize(2);
            display_print(temp_char);

            temp_score = temp_score/10;

        }

        G8RTOS_SignalSemaphore(&sem_SPI);

        score += 100;

        prev_score = score;

        temp_score = score;

        G8RTOS_WaitSemaphore(&sem_SPI);

        for (int i = 0; i < 5; i++){

            digit = temp_score % 10;
            temp_char = digit + '0';
            display_setCursor(180 - i*11, 270);
            display_setTextColor(0xFFFF);
            display_setTextSize(2);
            display_print(temp_char);

            temp_score = temp_score/10;

        }

        G8RTOS_SignalSemaphore(&sem_SPI);


        sleep(500);

    }
}

void State(void){

    uint16_t temp_score;
    uint16_t prev_score = 0;
    uint16_t digit;
    char temp_char;

    while(1){
        // Game Over
        if (state == 0){
            temp_score = prev_score;

            G8RTOS_WaitSemaphore(&sem_SPI);

            for(int i = 0; i < 6; i++){
                display_setCursor(69 + i*11, 270);
                display_setTextColor(0xFFFF);
                display_setTextSize(2);
                display_print(score_char[i]);
            }

            for (int i = 0; i < 5; i++){

                digit = temp_score % 10;
                temp_char = digit + '0';
                display_setCursor(180 - i*11, 270);
                display_setTextColor(0);
                display_setTextSize(2);
                display_print(temp_char);

                temp_score = temp_score/10;

            }

            G8RTOS_SignalSemaphore(&sem_SPI);

            score += 100;

            prev_score = score;

            temp_score = score;

            G8RTOS_WaitSemaphore(&sem_SPI);

            for (int i = 0; i < 5; i++){

                digit = temp_score % 10;
                temp_char = digit + '0';
                display_setCursor(180 - i*11, 270);
                display_setTextColor(0xFFFF);
                display_setTextSize(2);
                display_print(temp_char);

                temp_score = temp_score/10;

            }

            G8RTOS_SignalSemaphore(&sem_SPI);


            sleep(500);
        }
        if (state == 1){

//            G8RTOS_WaitSemaphore(&sem_SPI);
//            ST7789_Fill(ST7789_RED);
//            G8RTOS_SignalSemaphore(&sem_SPI);

            for(int i = 0; i < 9; i++){
                display_setCursor(80 + i*11, 290);
                display_setTextColor(ST7789_RED);
                display_setTextSize(2);
                display_print(game_char[i]);
            }

            for(int i = 0; i < 6; i++){
                display_setCursor(69 + i*11, 270);
                display_setTextColor(0xFFFF);
                display_setTextSize(2);
                display_print(score_char[i]);
            }

            temp_score = score;

            for (int i = 0; i < 5; i++){

                digit = temp_score % 10;
                temp_char = digit + '0';
                display_setCursor(180 - i*11, 270);
                display_setTextColor(0xFFFF);
                display_setTextSize(2);
                display_print(temp_char);

                temp_score = temp_score/10;


            }

            score = 0;


            while (state == 1);

        }
        // Start Screen
        else if (state == 2){

            for (int i = 0; i < 3; i++){
                display_setCursor(75 + i*35, 200);
                display_setTextColor(0xFFFF);
                display_setTextSize(5);
                display_print(run_char[i]);
            }

            for (int i = 0; i < 15; i++){
                display_setCursor(40 + i*11, 130);
                display_setTextColor(0xFFFF);
                display_setTextSize(2);
                display_print(name_char[i]);
            }

            while(state == 2);

        }
    }
}

void Draw_Border(void){

    while(1){

        while(state != 0){}

        if (render_border == 1){
            G8RTOS_WaitSemaphore(&sem_SPI);

            // LEFT
            ST7789_DrawRectangle(BottomRightX, BottomRightY, 10, 210, ST7789_BLACK);

            ST7789_DrawRectangle(BottomRightX, BottomRightY, 10, 210, left_color);

            // RIGHT
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 10, 210, ST7789_BLACK);

            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 10, 210, right_color);

            // TOP
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 200, 10, ST7789_BLACK);

            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 210, 10, top_color);

            // BOTTOM
            ST7789_DrawRectangle(TopLeftX, TopLeftY, 210, 10, ST7789_BLACK);

            ST7789_DrawRectangle(TopLeftX, TopLeftY, 210, 10, bottom_color);


            G8RTOS_SignalSemaphore(&sem_SPI);

            render_border = 0;
        }


        sleep(50);

    }

}

void Draw_Obstacles(void){

    while(1){

        if(render_obstacles == 1){

            G8RTOS_WaitSemaphore(&sem_SPI);


            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    if (obstacle_temp[j][i] == 1){
                        ST7789_DrawRectangle(i*10 + 30, (18 - j)*10 + 50, 10, 10, 0);
                    }


                }
            }

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    if (obstacle_bm[j][i] == 1){
                        ST7789_DrawRectangle(i*10 + 30, (18 - j)*10 + 50, 10, 10, 0xFFFF);
                    }


                }
            }


            G8RTOS_SignalSemaphore(&sem_SPI);

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){
                    obstacle_temp[i][j] = obstacle_bm[i][j];
                }
            }

            render_obstacles = 0;


        }
        sleep(50);

    }

}

void Animation(void){

    uint8_t TRX_End;
    uint8_t TLX_End;
    uint8_t BRX_End;
    uint8_t BLX_End;
    uint8_t playerX_End;

    uint8_t TRY_End;
    uint8_t TLY_End;
    uint8_t BRY_End;
    uint8_t BLY_End;
    uint8_t playerY_End;

    uint16_t temp;
    uint16_t e_temp;

//    int16_t change1X;
//    int16_t change1Y;
//
//    int16_t change2X;
//    int16_t change2Y;

    while(1){

        joystick_flag = 0;

        if (start_animation == 1){



            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    if (obstacle_temp[j][i] == 1){
                        ST7789_DrawRectangle(i*10 + 30, (18 - j)*10 + 50, 10, 10, 0);
                    }


                }
            }

            // LEFT
            ST7789_DrawRectangle(BottomRightX, BottomRightY, 10, 210, ST7789_BLACK);

            // RIGHT
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 10, 210, ST7789_BLACK);

            // TOP
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 200, 10, ST7789_BLACK);

            // BOTTOM
            ST7789_DrawRectangle(TopLeftX, TopLeftY, 210, 10, ST7789_BLACK);

            ST7789_DrawRectangle(playerX, playerY, 4, 10, ST7789_BLACK);

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    ST7789_DrawRectangle(enemy[i].prev_x, enemy[i].prev_y, 5, 5, 0);
                    e_temp = enemy[i].x;
                    enemy[i].x_end = enemy[i].y - 20;
                    enemy[i].y_end = 240 - e_temp + 20;

                }

            }
            // Hit a right wall

            TRX_End = BottomRightX;
            TRY_End = BottomRightY;

            TLX_End = TopRightX;
            TLY_End = TopRightY;

            BRX_End = BottomLeftX;
            BRY_End = BottomLeftY;

            BLX_End = TopLeftX;
            BLY_End = TopLeftY;

            playerX_End = 35 + playerY - 50;
            playerY_End = 50;



            for(int i = 0; i < 10; i++){

                // Right Wall

                // Top Right Moves to Top Bottom
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, ST7789_GREEN);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0xFFFF);

                move_line(BottomRightX, BottomRightY, BRX_End, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, top_color);

                // Top Wall

                // Top Left Moves to Top Right
                // Top Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, left_color);

                // Left Wall

                // Top Left Moves to Top Right
                // Bottom Left Moves to Top Right
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End + 10, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, bottom_color);

                // Bottom Wall

                // Bottom Left Moves to Top Right
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, right_color);

                SysCtlDelay(80000000/500);

                // Clear Walls

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, 0);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0);

                move_line(BottomRightX, BottomRightY, BRX_End, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, 0);
                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, 0);
                move_line(TopLeftX, TopLeftY, TLX_End + 10, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, 0);
                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, 0);
            }

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    e_temp = enemy[i].x;
                    enemy[i].x = enemy[i].y - 20;
                    enemy[i].y = 240 - e_temp + 20;

                }

            }

            playerX = 35 + playerY - 50;
            playerY = 50;

            start_animation = 0;
            render_border = 1;
            render_obstacles = 1;





        }
        else if(start_animation == 2){

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    if (obstacle_temp[j][i] == 1){
                        ST7789_DrawRectangle(i*10 + 30, (18 - j)*10 + 50, 10, 10, 0);
                    }


                }
            }

            // LEFT
            ST7789_DrawRectangle(BottomRightX, BottomRightY, 10, 210, ST7789_BLACK);

            // RIGHT
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 10, 210, ST7789_BLACK);

            // TOP
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 200, 10, ST7789_BLACK);

            // BOTTOM
            ST7789_DrawRectangle(TopLeftX, TopLeftY, 210, 10, ST7789_BLACK);

            ST7789_DrawRectangle(playerX, playerY, 4, 10, ST7789_BLACK);

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    ST7789_DrawRectangle(enemy[i].prev_x, enemy[i].prev_y, 5, 5, 0);
                    e_temp = enemy[i].x;
                    enemy[i].x_end = 215 - enemy[i].y + 50;
                    enemy[i].y_end = e_temp + 20;

                }

            }

            // Hit a left wall

            TRX_End = TopLeftX;
            TRY_End = TopLeftY;

            TLX_End = BottomLeftX;
            TLY_End = BottomLeftY;

            BRX_End = TopRightX;
            BRY_End = TopRightY;

            BLX_End = BottomRightX;
            BLY_End = BottomRightY;

            playerX_End = 215 - playerY + 50;
            playerY_End = 50;


            for(int i = 0; i < 10; i++){

                // Right Wall

                // Top Right Moves to Top Bottom
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, ST7789_GREEN);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0xFFFF);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, bottom_color);

                // Top Wall

                // Top Left Moves to Top Right
                // Top Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, right_color);

                // Left Wall

                // Top Left Moves to Top Right
                // Bottom Left Moves to Top Right
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, top_color);

                // Bottom Wall

                // Bottom Left Moves to Top Right
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, left_color);

                SysCtlDelay(80000000/500);

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, 0);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, 0);

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, 0);

                // Clear Walls
            }

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    e_temp = enemy[i].x;
                    enemy[i].x = 215 - enemy[i].y + 50;
                    enemy[i].y = e_temp + 20;

                }

            }

            playerX = 215 - playerY + 50;
            playerY = 50;

            start_animation = 0;
            render_border = 1;
            render_obstacles = 1;

        }
        else if(start_animation == 3){


            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    if (obstacle_temp[j][i] == 1){
                        ST7789_DrawRectangle(i*10 + 30, (18 - j)*10 + 50, 10, 10, 0);
                    }


                }
            }

            // LEFT
            ST7789_DrawRectangle(BottomRightX, BottomRightY, 10, 210, ST7789_BLACK);

            // RIGHT
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 10, 210, ST7789_BLACK);

            // TOP
            ST7789_DrawRectangle(BottomLeftX, BottomLeftY, 200, 10, ST7789_BLACK);

            // BOTTOM
            ST7789_DrawRectangle(TopLeftX, TopLeftY, 210, 10, ST7789_BLACK);

            ST7789_DrawRectangle(playerX, playerY, 4, 10, ST7789_BLACK);

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    ST7789_DrawRectangle(enemy[i].prev_x, enemy[i].prev_y, 5, 5, 0);
                    e_temp = enemy[i].x;
                    enemy[i].x_end = (220 - enemy[i].x + 30)/2;
                    enemy[i].y_end = (240 - (enemy[i].y - 50))/2;

                }

            }

            // Hit a left wall

            TRX_End = TopLeftX;
            TRY_End = TopLeftY;

            TLX_End = BottomLeftX;
            TLY_End = BottomLeftY;

            BRX_End = TopRightX;
            BRY_End = TopRightY;

            BLX_End = BottomRightX;
            BLY_End = BottomRightY;

            temp = 220 - playerX;

            playerX_End = 30;
            playerY_End = 120;

            for(int i = 0; i < 10; i++){

                // Right Wall

                // Top Right Moves to Top Bottom
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation
                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, ST7789_GREEN);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0xFFFF);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, bottom_color);

                // Top Wall

                // Top Left Moves to Top Right
                // Top Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, right_color);

                // Left Wall

                // Top Left Moves to Top Right
                // Bottom Left Moves to Top Right
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, top_color);

                // Bottom Wall

                // Bottom Left Moves to Top Right
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, left_color);

                SysCtlDelay(80000000/500);

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, 0);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, 0);

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, 0);

                // Clear Walls
            }

            // Hit a left wall

            TRX_End = TopLeftX;
            TRY_End = TopLeftY;

            TLX_End = BottomLeftX;
            TLY_End = BottomLeftY;

            BRX_End = TopRightX;
            BRY_End = TopRightY;

            BLX_End = BottomRightX;
            BLY_End = BottomRightY;

            playerX = 30;
            playerY = 120;

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    e_temp = enemy[i].x;
                    enemy[i].x_end = 220 - enemy[i].x + 30;
                    enemy[i].y_end = 240 - (enemy[i].y - 50);

                    enemy[i].x = (220 - enemy[i].x + 30)/2;
                    enemy[i].y = (240 - (enemy[i].y - 50))/2;

                }

            }

            playerX_End = 30 + temp;
            playerY_End = 50;

            for(int i = 0; i < 10; i++){

                // Right Wall

                // Top Right Moves to Top Bottom
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, ST7789_GREEN);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0xFFFF);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, bottom_color);

                // Top Wall

                // Top Left Moves to Top Right
                // Top Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, right_color);

                // Left Wall

                // Top Left Moves to Top Right
                // Bottom Left Moves to Top Right
                // Draw Lines to show this animation

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, top_color);

                // Bottom Wall

                // Bottom Left Moves to Top Right
                // Bottom Right Moves to Bottom Left
                // Draw Lines to show this animation

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, left_color);

                SysCtlDelay(80000000/500);

                for(int j = 0; j < 3; j++){

                    if(enemy[j].alive == 1){

                        move_line(enemy[j].x, enemy[j].y, enemy[j].x_end, enemy[j].y_end, enemy[j].x, enemy[j].y + 5, enemy[j].x_end, enemy[j].y_end + 5, 1, 5, i, 0);

                    }

                }

                move_line(playerX, playerY, playerX_End, playerY_End, playerX, playerY + 10, playerX_End, playerY_End + 10, 1, 4, i, 0);

                move_line(BottomRightX, BottomRightY, BRX_End + 10, BRY_End, TopRightX, TopRightY, TRX_End, TRY_End, 1, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, TopRightX, TopRightY, TRX_End, TRY_End, 2, 10, i, 0);

                move_line(TopLeftX, TopLeftY, TLX_End, TLY_End, BottomLeftX, BottomLeftY, BLX_End, BLY_End, 1, 10, i, 0);

                move_line(BottomLeftX, BottomLeftY, BLX_End, BLY_End, BottomRightX, BottomRightY, BRX_End, BRY_End, 2, 10, i, 0);

                // Clear Walls
            }

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    e_temp = enemy[i].x;
                    enemy[i].x = enemy[i].x_end;
                    enemy[i].y = enemy[i].y_end;

                }

            }

            playerX = 30 + temp;
            playerY = 50;

            start_animation = 0;
            render_border = 1;
            render_obstacles = 1;

        }

        joystick_flag = 1;

        sleep(50);

    }



}

void move_line(uint16_t Start1X, uint16_t Start1Y, uint16_t End1X, uint16_t End1Y, uint16_t Start2X, uint16_t Start2Y, uint16_t End2X, uint16_t End2Y, uint16_t direction, int16_t size, uint16_t i, uint16_t color){

    int16_t change1X;
    int16_t change1Y;

    int16_t change2X;
    int16_t change2Y;

    // Vertical to Horizontal
    if (direction == 1){
        change1X = (End1X - Start1X)/10;
        change1Y = (End1Y - Start1Y)/10;

        change2X = (End2X - Start2X)/10;
        change2Y = (End2Y - Start2Y)/10;


        ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), color);

        if (i != 4){
            // Horizontal
            ST7789_DrawLine(Start1X + change1X*(i+1) - size, Start1Y + change1Y*(i+1) + size, Start2X + change2X*(i+1) - size, Start2Y + change2Y*(i+1) + size, color);
            // Vertical Connectors
            ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start1X + change1X*(i+1) - size, Start1Y + change1Y*(i+1) + size, color);
            ST7789_DrawLine(Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), Start2X + change2X*(i+1) - size, Start2Y + change2Y*(i+1) + size, color);
        }
        else{
            // Horizontal
            ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1) + size, Start2X + change2X*(i+1), Start2Y + change2Y*(i+1) + size, color);
            // Vertical Connectors
            ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start1X + change1X*(i+1), Start1Y + change1Y*(i+1) + size, color);
            ST7789_DrawLine(Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), Start2X + change2X*(i+1), Start2Y + change2Y*(i+1) + size, color);
        }




    }
    // Horizontal to Vertical
    else if(direction == 2){

        change1X = (End1X - Start1X)/10;
        change1Y = (End1Y - Start1Y)/10;

        change2X = (End2X - Start2X)/10;
        change2Y = (End2Y - Start2Y)/10;

        ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), color);

        if (i != 4){
            // Horizontal
            ST7789_DrawLine(Start1X + change1X*(i+1) - size, Start1Y + change1Y*(i+1) + size, Start2X + change2X*(i+1) - size, Start2Y + change2Y*(i+1) + size, color);
            // Vertical Connectors
            ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start1X + change1X*(i+1) - size, Start1Y + change1Y*(i+1) + size, color);
            ST7789_DrawLine(Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), Start2X + change2X*(i+1) - size, Start2Y + change2Y*(i+1) + size, color);
        }
        else{
            // Horizontal
            ST7789_DrawLine(Start1X + change1X*(i+1) + size, Start1Y + change1Y*(i+1), Start2X + change2X*(i+1) + size, Start2Y + change2Y*(i+1), color);
            // Vertical Connectors
            ST7789_DrawLine(Start1X + change1X*(i+1), Start1Y + change1Y*(i+1), Start1X + change1X*(i+1) + size, Start1Y + change1Y*(i+1), color);
            ST7789_DrawLine(Start2X + change2X*(i+1), Start2Y + change2Y*(i+1), Start2X + change2X*(i+1) + size, Start2Y + change2Y*(i+1), color);
        }


    }


}

void Draw_Player(void){

    uint16_t prev_x = playerX;
    uint16_t prev_y = playerY;

    while(1){

        while(state != 0){}

        G8RTOS_WaitSemaphore(&sem_SPI);
        ST7789_DrawRectangle(prev_x, prev_y, 4, 10, ST7789_BLACK);

        ST7789_DrawRectangle(playerX, playerY, 4, 10, ST7789_WHITE);

        prev_x = playerX;
        prev_y = playerY;

        G8RTOS_SignalSemaphore(&sem_SPI);

        sleep(20);

    }

}


void Move_Player(void){

    uint32_t temp;
    uint16_t tempX;
    uint16_t tempY;
    uint16_t joystickX;
    uint16_t joystickY;
    float normalX;
    float normalY;

    while(1){
        temp = G8RTOS_ReadFIFO(0);
        joystickX = (temp >> 16);
        joystickY = temp;

        normalX = 2.0*(((float)(joystickX)) / 4096.0) - 1.0;
        normalY = 2.0*(((float)(joystickY)) / 4096.0) - 1.0;

        if(normalX > 0.3){
            normalX = normalX * -15;
        }
        else if(normalX < -0.3){
            normalX = normalX * -15;
        }
        else{
            normalX = 0;
        }

        tempX = playerX + normalX;
        tempY = playerY + playerV;

        // Vertical Movement

        // Checks if we bump out head
        if(obstacle_bm[18 - (tempY - 7 - 50)/10 - 1][(playerX - 30)/10] == 1 && (playerY - 3 < ((tempY - 50)/10 * 10 + 50 - 11))){
            playerY = (tempY - 50)/10 * 10 + 50 - 11;
            playerV = 0;
        }
        else if (obstacle_bm[18 - (tempY - 50)/10 - 1][(playerX - 30)/10] == 1 && (playerY - 3 < ((tempY - 50)/10 * 10 + 50 - 1))){
            playerY = (tempY - 50)/10 * 10 + 50 - 1;
            playerV = 0;
        }
        // Makes sure we don't phase though

        // Checks if block is below
        else if (obstacle_bm[18 - (tempY + 7 - 50)/10][(playerX - 30)/10] == 1 && (playerY + 3 > ((tempY - 50)/10 * 10 + 50 + 21))){
            playerY = (tempY - 50)/10 * 10 + 50 + 21;
            playerV = 0;
        }
        else if (obstacle_bm[18 - (tempY - 50)/10][(playerX - 30)/10] == 1 && (playerY + 3 > ((tempY - 50)/10 * 10 + 50 + 11))){
            playerY = (tempY - 50)/10 * 10 + 50 + 11;
            playerV = 0;
        }
        // Makes sure we don't phase through a block

        // We are flying
        else if (tempY <= 50){
            playerY = 50;
            playerV = 0;
        }
        // We on the floor
        else{
            playerV += playerA;
            playerY = tempY;
            if (playerV < -15){
                playerV = -15;
            }
        }

        // Horizontal Movement


        // Check Right
        if (obstacle_bm[18 - (playerY - 50)/10 - 1][(tempX - 10 - 30)/10] == 1 && (playerX - 7 < ((tempX - 30)/10 * 10 + 30 - 15))){
            playerX = (tempX - 30)/10 * 10 + 30 - 15;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10 - 1][(tempX - 30)/10] == 1 && (playerX - 7 < ((tempX - 30)/10 * 10 + 30 - 5))){
            playerX = (tempX - 30)/10 * 10 + 30 - 5;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10][(tempX - 10 - 30)/10] == 1 && (playerX - 7 < ((tempX - 30)/10 * 10 + 30 - 15))){

            playerX = (tempX - 30)/10 * 10 + 30 - 15;
            //playerX = 150;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10][(tempX - 30)/10] == 1 && (playerX - 7 < ((tempX - 30)/10 * 10 + 30 - 5))){

            playerX = (tempX - 30)/10 * 10 + 30 - 5;
            //playerX = 150;

        }
        // Check Left
        else if (obstacle_bm[18 - (playerY - 50)/10 - 1][(tempX + 10 - 30)/10] == 1 && (playerX + 2 > ((tempX - 30)/10 * 10 + 30 + 21))){
            playerX = (tempX - 30)/10 * 10 + 30 + 21;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10 - 1][(tempX - 30)/10] == 1 && (playerX + 2 > ((tempX - 30)/10 * 10 + 30 + 11))){
            playerX = (tempX - 30)/10 * 10 + 30 + 11;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10][(tempX + 10 - 30)/10] == 1 && (playerX + 2 > ((tempX - 30)/10 * 10 + 30 + 21))){

            playerX = (tempX - 30)/10 * 10 + 30 + 21;
            //playerX = 150;

        }
        else if (obstacle_bm[18 - (playerY - 50)/10][(tempX - 30)/10] == 1 && (playerX + 2 > ((tempX - 30)/10 * 10 + 30 + 11))){

            playerX = (tempX - 30)/10 * 10 + 30 + 11;
            //playerX = 150;

        }

        else{
            playerX = tempX;
        }



        if(playerX >= BottomRightX - 4){

            uint16_t temp = bottom_color;

            bottom_color = right_color;
            right_color = top_color;
            top_color = left_color;
            left_color = temp;

//            playerX = 35 + playerY - 50;
//            playerY = 50;


//            for (int i = 0; i < 3; i++){
//                if (enemy[i].alive == 1){
//                    temp = enemy[i].x;
//                    enemy[i].x = enemy[i].y - 20;
//                    enemy[i].y = 240 - temp + 20;
//                }
//            }

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    obstacle_bm[i][j] = obstacle_temp[18 - j][i];

                }
            }

            start_animation = 1;




        }
        else if(playerX <= (BottomLeftX + 10)){

            uint16_t temp = bottom_color;

            bottom_color = left_color;
            left_color = top_color;
            top_color = right_color;
            right_color = temp;

//            playerX = 215 - playerY + 50;
//            playerY = 50;

//            for (int i = 0; i < 3; i++){
//                if (enemy[i].alive == 1){
//                    temp = enemy[i].x;
//                    enemy[i].x = 215 - enemy[i].y + 50;
//                    enemy[i].y = temp + 20;
//                }
//            }

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    obstacle_bm[i][j] = obstacle_temp[j][18 - i];

                }
            }

            start_animation = 2;

        }
        else if(playerY >= (TopRightY - 12)){

            uint16_t temp = bottom_color;

            bottom_color = top_color;
            top_color = temp;

//            playerY = 50;

//            for (int i = 0; i < 3; i++){
//                if (enemy[i].alive == 1){
//                    enemy[i].x = 220 - enemy[i].x + 30;
//                    enemy[i].y = 240 - (enemy[i].y - 50) ;
//                }
//            }

            for(int i = 0; i < 19; i++){
                for(int j = 0; j < 19; j++){

                    obstacle_bm[i][j] = obstacle_temp[18 - i][18 - j];

                }
            }

            start_animation = 3;


        }
        sleep(10);

    }

}

uint8_t h(position_t start, position_t end)
{
    return abs(((int8_t)start.x) - end.x) + abs(((int8_t)start.y) - end.y);
}

// A* algorithm
const uint8_t MAP_SIZE = 19;
a_star_info_t map[MAP_SIZE][MAP_SIZE];
position_t open_set[MAP_SIZE * MAP_SIZE];

position_t astar(position_t start, position_t end)
{

    if (start.x == end.x && end.y == start.y){
        return start;
    }

    uint8_t open_set_size = 0;

    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            map[i][j].score = -1;
            map[i][j].distance = -1;
            map[i][j].came_from.x = -1;
            map[i][j].came_from.y = -1;
        }
    }

    map[start.x][start.y].distance = 0;
    map[start.x][start.y].score = h(start, end);

    open_set[open_set_size++] = start;

    while (open_set_size > 0)
    {
        a_star_info_t *current = NULL;
        position_t current_pos = {0, 0};

        // Find the node in open_set with the lowest score
        for (int i = 0; i < open_set_size; i++)
        {
            if (current == NULL || map[open_set[i].y][open_set[i].x].score < current->score)
            {
                current = &map[open_set[i].y][open_set[i].x];
                current_pos = open_set[i];
            }
        }

        // If we found the end, reconstruct the path
        if (current_pos.x == end.x && current_pos.y == end.y)
        {
            while (current->came_from.x != start.x || current->came_from.y != start.y)
            {
                current_pos = current->came_from;
                current = &map[current->came_from.y][current->came_from.x];
            }
            return current_pos;
        }

        // Remove current from open_set
        for (int i = 0; i < open_set_size; i++)
        {
            if (open_set[i].x == current_pos.x && open_set[i].y == current_pos.y)
            {
                for (int j = i; j < open_set_size - 1; j++)
                {
                    open_set[j] = open_set[j + 1];
                }
                open_set_size--;
                break;
            }
        }

        // Check neighbors
        position_t neighbors[4] = {
            {current_pos.x - 1, current_pos.y},
            {current_pos.x + 1, current_pos.y},
            {current_pos.x, current_pos.y - 1},
            {current_pos.x, current_pos.y + 1}};

        for (int i = 0; i < 4; i++)
        {
            position_t neighbor = neighbors[i];

            // Check if the neighbor is in the map
            if (neighbor.x >= MAP_SIZE || neighbor.y >= MAP_SIZE)
            {
                continue;
            }

            // Check if the neighbor is an obstacle
            if (obstacle_bm[neighbor.y][neighbor.x])
            {
                continue;
            }


            uint8_t tentative_distance = current->distance + 1;
            if (tentative_distance < map[neighbor.y][neighbor.x].distance)
            {
                map[neighbor.y][neighbor.x].distance = tentative_distance;
                map[neighbor.y][neighbor.x].score = tentative_distance + h(neighbor, end);
                map[neighbor.y][neighbor.x].came_from = current_pos;
                open_set[open_set_size++] = neighbor;
            }
        }
    }
}


uint16_t Get_H(position_t p){

    position_t temp_p;
    temp_p.x = playerX;
    temp_p.y = playerY;
    temp_p = Get_Position(temp_p);

    int8_t dx = temp_p.x;
    dx = abs(dx - p.x);
    int8_t dy = temp_p.y;
    dy = abs(dy - p.y);

    return dx + dy;



}

position_t Get_Position(position_t p){

    p.x = (p.x - 30)/10;
    p.y = 18 - (p.y - 50)/10;

    return p;
}



void Enemy(void){

    for(int i = 0; i < 3; i++){
        enemy[i].x = (rand() % 150) + 40;
        enemy[i].y = (rand() % 100) + 70;
        enemy[i].prev_x = enemy[i].x;
        enemy[i].prev_y = enemy[i].y;
    }

    int chance = 0;

    enemy[0].alive = 1;


    //int speed = (rand() % 25 + 40);
    int speed = 100;
    while(1){

            while(state != 0){}

            for (int i = 0; i < 3; i++){

                if (enemy[i].alive == 1){

                    // Draw The Enemy

                    G8RTOS_WaitSemaphore(&sem_SPI);
                    ST7789_DrawRectangle(enemy[i].prev_x, enemy[i].prev_y, 5, 5, ST7789_BLACK);

                    ST7789_DrawRectangle(enemy[i].x, enemy[i].y, 5, 5, ST7789_GREEN);
                    G8RTOS_SignalSemaphore(&sem_SPI);




                    enemy[i].prev_x = enemy[i].x;
                    enemy[i].prev_y = enemy[i].y;

                    // Check if there is a collision with the player

                    if ((playerX - enemy[i].x <= 5) && (playerY - enemy[i].y <= 5) && (enemy[i].x - playerX <= 4) && (enemy[i].y - playerY <= 10)){
                        state = 1;
                    }

                    // Update position using temp position and collision Detection

                    // Update temp position

                    // A* Algorithm

                    // Update the previous position
                    position_t start_position;
                    start_position.x = enemy[i].x;
                    start_position.y = enemy[i].y;
                    start_position = Get_Position(start_position);

                    position_t end_position;
                    end_position.x = playerX;
                    end_position.y = playerY;
                    end_position = Get_Position(end_position);

                    position_t position;
                    position = astar(start_position, end_position);

                    enemy[i].tempX = (position.x * 10) + 30;
                    enemy[i].tempY = ((18 - position.y) * 10) + 50;

//                    if(enemy[i].tempX > enemy[i].x){
//                        enemy[i].tempX -= 5;
//                    }
//                    else if(enemy[i].tempX < enemy[i].x){
//                        enemy[i].tempX += 5;
//                    }
//                    else if(enemy[i].tempY > enemy[i].y){
//                        enemy[i].tempY -= 5;
//                    }
//                    else if(enemy[i].tempY < enemy[i].y){
//                        enemy[i].tempY += 5;
//                    }

                    // Updates Temp Position
//                    if(playerX < enemy[i].x){
//                        enemy[i].tempX = enemy[i].x - 5;
//                    }
//                    else{
//                        enemy[i].tempX = enemy[i].x + 5;
//                    }
//
//                    if(playerY < enemy[i].y){
//                        enemy[i].tempY = enemy[i].y - 5;
//                    }
//                    else{
//                        enemy[i].tempY = enemy[i].y + 5;
//                    }

                    // Collision Detection


                    // Vertical Movement

                    // Checks if we bump out head
//                    if(obstacle_bm[18 - (enemy[i].tempY - 10 - 50)/10][(enemy[i].x - 30)/10] == 1 && (enemy[i].y - 5 < ((enemy[i].tempY - 50)/10 * 10 + 50 - 15))){
//                        enemy[i].y = (enemy[i].tempY - 50)/10 * 10 + 50 - 15;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].tempY - 50)/10][(enemy[i].x - 30)/10] == 1 && (enemy[i].y - 5 < ((enemy[i].tempY - 50)/10 * 10 + 50 - 5))){
//                        enemy[i].y = (enemy[i].tempY - 50)/10 * 10 + 50 - 5;
//
//                    }
//                    // Makes sure we don't phase though
//
//                    // Checks if block is below
//                    else if (obstacle_bm[18 - (enemy[i].tempY + 7 - 50)/10][(enemy[i].x - 30)/10] == 1 && (enemy[i].y + 5 > ((enemy[i].tempY - 50)/10 * 10 + 50 + 21))){
//                        enemy[i].y = (enemy[i].tempY - 50)/10 * 10 + 50 + 21;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].tempY - 50)/10][(enemy[i].x - 30)/10] == 1 && (enemy[i].y + 5 > ((enemy[i].tempY - 50)/10 * 10 + 50 + 11))){
//                        enemy[i].y = (enemy[i].tempY - 50)/10 * 10 + 50 + 11;
//
//                    }
//                    // Makes sure we don't phase through a block
//
//                    // We are flying
//                    else if (enemy[i].tempY <= 50){
//                        enemy[i].y = 50;
//                    }
//                    // We on the floor
//                    else{
//
//                        enemy[i].y = enemy[i].tempY;
//
//                    }
//
//                    // Horizontal Movement
//
//                    // Check Right
//                    if (obstacle_bm[18 - (enemy[i].y - 50)/10 - 1][(enemy[i].tempX - 10 - 30)/10] == 1 && (enemy[i].x - 6 < ((enemy[i].tempX - 30)/10 * 10 + 30 - 15))){
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 - 15;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10 - 1][(enemy[i].tempX - 30)/10] == 1 && (enemy[i].x - 6 < ((enemy[i].tempX - 30)/10 * 10 + 30 - 5))){
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 - 5;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10][(enemy[i].tempX - 10 - 30)/10] == 1 && (enemy[i].x - 6 < ((enemy[i].tempX - 30)/10 * 10 + 30 - 15))){
//
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 - 15;
//                        //playerX = 150;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10][(enemy[i].tempX - 30)/10] == 1 && (enemy[i].x - 6 < ((enemy[i].tempX - 30)/10 * 10 + 30 - 5))){
//
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 - 5;
//                        //playerX = 150;
//
//                    }
//                    // Check Left
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10 - 1][(enemy[i].tempX + 10 - 30)/10] == 1 && (enemy[i].x + 6 > ((enemy[i].tempX - 30)/10 * 10 + 30 + 21))){
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 + 21;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10 - 1][(enemy[i].tempX - 30)/10] == 1 && (enemy[i].x + 6 > ((enemy[i].tempX - 30)/10 * 10 + 30 + 11))){
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 + 11;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10][(enemy[i].tempX + 10 - 30)/10] == 1 && (enemy[i].x + 6 > ((enemy[i].tempX - 30)/10 * 10 + 30 + 21))){
//
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 + 21;
//                        //playerX = 150;
//
//                    }
//                    else if (obstacle_bm[18 - (enemy[i].y - 50)/10][(enemy[i].tempX - 30)/10] == 1 && (enemy[i].x + 6 > ((enemy[i].tempX - 30)/10 * 10 + 30 + 11))){
//
//                        enemy[i].x = (enemy[i].tempX - 30)/10 * 10 + 30 + 11;
//                        //playerX = 150;
//
//                    }
//
//                    else{
//                        enemy[i].x = enemy[i].tempX;
//                    }

                    enemy[i].x = enemy[i].tempX;
                    enemy[i].y = enemy[i].tempY;



                    chance = rand() % 300;
                    if (chance == 10){
                        enemy[i].alive = 0;
                        G8RTOS_WaitSemaphore(&sem_SPI);
                        ST7789_DrawRectangle(enemy[i].prev_x, enemy[i].prev_y, 5, 5, ST7789_BLACK);
                        G8RTOS_SignalSemaphore(&sem_SPI);
                        numEnemy--;
                    }



                }

            }
            sleep(speed);



            // collision detection




        }


}

void Add_Enemy(void){

    if(numEnemy < 3){

        for(int i = 0; i < 3; i++){
            if(enemy[i].alive == 0){
                enemy[i].x = (rand() % 150) + 40;
                enemy[i].y = (rand() % 100) + 70;
                enemy[i].prev_x = enemy[i].x;
                enemy[i].prev_y = enemy[i].y;
                enemy[i].alive = 1;
                break;
            }
        }
        numEnemy++;

    }

}

void Get_Joystick(void) {
    // Read the joystick
    if (joystick_flag == 1 && state == 0){
        uint32_t temp = JOYSTICK_GetXY();
        // Send through FIFO.
        G8RTOS_WriteFIFO(0, temp);
    }
}

void Fix_Screen(void){
    if (state == 0){
        render_obstacles = 1;
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

            if (state == 1){


                playerX = 40;
                playerY = 50;
                for(int i = 0; i < 3; i++){
                    enemy[i].x = (rand() % 150) + 40;
                    enemy[i].y = (rand() % 100) + 70;
                    enemy[i].prev_x = enemy[0].x;
                    enemy[i].prev_y = enemy[0].y;
                }
                enemy[0].alive = 1;
                enemy[1].alive = 0;
                enemy[2].alive = 0;
                numEnemy = 1;
                ST7789_Fill(ST7789_BLACK);
                render_border = 1;
                render_obstacles = 1;
                state = 0;

            }


        }
        else if(buttonVal == 253){
            UARTprintf("Button 2 Pressed\n");
            playerV = 15;
        }
        else if(buttonVal == 247){

            if (state == 2){

                playerX = 40;
                playerY = 50;
                for(int i = 0; i < 3; i++){
                    enemy[i].x = (rand() % 150) + 40;
                    enemy[i].y = (rand() % 100) + 70;
                    enemy[i].prev_x = enemy[0].x;
                    enemy[i].prev_y = enemy[0].y;
                }
                enemy[0].alive = 1;
                enemy[1].alive = 0;
                enemy[2].alive = 0;
                numEnemy = 1;
                ST7789_Fill(ST7789_BLACK);
                render_border = 1;
                render_obstacles = 1;
                state = 0;


            }
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
