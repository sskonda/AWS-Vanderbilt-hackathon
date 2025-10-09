//
//#ifndef FINAL_THREADS_H
//#define FINAL_THREADS_H
//
//// Includes
//
//#include "./G8RTOS/G8RTOS.h"
//
//// Sempahores
//
//semaphore_t sem_SPI;
//semaphore_t sem_GPIOE;
//semaphore_t sem_UART;
//semaphore_t sem_GPIOD;
//
//// Structures
//
//typedef struct position_t {
//    uint8_t x;
//    uint8_t y;
//} position_t;
//
//typedef struct a_star_info_t {
//
//    position_t came_from;
//    uint8_t score;
//    uint8_t distance;
//
//
//} a_star_info_t;
//
//typedef struct enemy_t {
//
//    uint16_t x;
//    uint16_t tempX;
//    uint16_t y;
//    uint16_t tempY;
//    uint16_t prev_x;
//    uint16_t prev_y;
//    uint16_t x_end;
//    uint16_t y_end;
//    uint8_t alive;
//
//} enemy_t;
//
//// Threads
//
//void Idle_Thread(void);
//
//void Score_Thread(void);
//
//void State(void);
//
//void Draw_Border(void);
//
//void Draw_Obstacles(void);
//
//void Animation(void);
//
//void Draw_Player(void);
//
//void Move_Player(void);
//
//void Enemy(void);
//
//void Read_Buttons(void);
//
//
//
//
//
//// Periodic Threads
//
//void Add_Enemy(void);
//
//void Get_Joystick(void);
//
//void Fix_Screen(void);
//
//// Aperiodic Threads
//
//void GPIOE_Handler(void);
//
//// Functions
//
//void move_line(uint16_t Start1X,uint16_t Start1Y,uint16_t End1X,uint16_t End1Y,uint16_t Start2X,uint16_t Start2Y,uint16_t End2X, uint16_t End2Y,uint16_t direction,int16_t size,uint16_t i, uint16_t color);
//
//position_t astar(position_t start, position_t end);
//
//position_t Get_Position(position_t p);
//
//uint8_t h(position_t start, position_t end);
//
//
//
//#endif /* FINAL_THREADS_H_ */
