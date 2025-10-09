#pragma once

#include "./G8RTOS/G8RTOS.h"
#include <math.h>

// Structs and defines

// gravity constant for 1g in raw sensor units
#define GRAVITY 16000.0f
#define PI 3.14159f

// Struct to hold tilt results
typedef struct {
    float forward;
    float side;
} Tilt_t;

typedef struct {
    int32_t px, py, pz;   /* integer world positions (units) */

    float fx, fy, fz;

    float thrust;
} ShipState;

/* Normalize a 3D vector */
static void normalize(float *x, float *y, float *z) {
    float len = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    if (len > 0.0f) {
        *x /= len;
        *y /= len;
        *z /= len;
    }
}

/* Cross product: a x b */
static void cross(float ax, float ay, float az,
           float bx, float by, float bz,
           float *rx, float *ry, float *rz) {
    *rx = ay * bz - az * by;
    *ry = az * bx - ax * bz;
    *rz = ax * by - ay * bx;
}

/* Rotate a vector around an arbitrary axis (unit vector) */
static void rotateAroundAxis(float *vx, float *vy, float *vz,
                      float ax, float ay, float az, float angle) {
    float cosA = cosf(angle);
    float sinA = sinf(angle);
    float dot = (*vx)*ax + (*vy)*ay + (*vz)*az;
    float rx = (*vx)*cosA + sinA * (ay*(*vz) - az*(*vy)) + ax*dot*(1 - cosA);
    float ry = (*vy)*cosA + sinA * (az*(*vx) - ax*(*vz)) + ay*dot*(1 - cosA);
    float rz = (*vz)*cosA + sinA * (ax*(*vy) - ay*(*vx)) + az*dot*(1 - cosA);
    *vx = rx; *vy = ry; *vz = rz;
}

/* Update ship orientation based on tilt */
static void applyTilt(ShipState *ship, Tilt_t tilt) {
    // Scale angles for stronger effect
    float pitchAngle = tilt.forward * 0.15f;  // pitch (forward tilt)
    float yawAngle   = tilt.side * 0.15f;     // yaw (side tilt)

    // 1. Compute local right vector = cross(forward, world up)
    float rx, ry, rz;
    cross(ship->fx, ship->fy, ship->fz, 0.0f, 1.0f, 0.0f, &rx, &ry, &rz);
    normalize(&rx, &ry, &rz);

    // 2. Apply pitch: rotate forward vector around local right axis
    rotateAroundAxis(&ship->fx, &ship->fy, &ship->fz, rx, ry, rz, pitchAngle);

    // 3. Apply yaw: rotate forward vector around world up axis
    rotateAroundAxis(&ship->fx, &ship->fy, &ship->fz, 0.0f, 1.0f, 0.0f, yawAngle);

    // Normalize after rotations
    normalize(&ship->fx, &ship->fy, &ship->fz);
}

/* Update ship position */
static void updateShipPosition(ShipState *ship, Tilt_t tilt) {
    applyTilt(ship, tilt);

    ship->px += (int32_t)(ship->fx * ship->thrust);
    ship->py += (int32_t)(ship->fy * ship->thrust);
    ship->pz += (int32_t)(ship->fz * ship->thrust);
}

//static inline void project3D(float x, float y, float z, int *out_x, int *out_y)
//{
//    const float l = 0.5f;
//    const float ca = 0.70710678f; // cos(45°)
//    const float sa = 0.70710678f; // sin(45°)
//
//    float px = x + l * z * ca;
//    float py = -y + l * z * sa; // negative y => up on screen
//
//    // simple round-to-nearest without roundf()
//    int rx = (int)(px + (px >= 0.0f ? 0.5f : -0.5f));
//    int ry = (int)(py + (py >= 0.0f ? 0.5f : -0.5f));
//
//    *out_x = rx;
//    *out_y = ry;
//}

static inline int fround_to_int(float v)
{
    return (int)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

/* same cabinet projection used previously */
static void project3D(float x, float y, float z, int *out_x, int *out_y)
{
    const float l  = 0.5f;
    const float ca = 0.70710678f; /* cos(45°) */
    const float sa = 0.70710678f; /* sin(45°) */

    float px = x + l * z * ca;
    float py = -y + l * z * sa; /* -y so positive Y in 3D goes up on screen */

    *out_x = fround_to_int(px);
    *out_y = fround_to_int(py);
}

semaphore_t sem_GPIOE;
semaphore_t sem_SPI;
semaphore_t sem_UART;
semaphore_t sem_I2C;

void Idle_Thread(void);

// BEAGLEBONE
void BeagleBone_Do(void);

// BUTTONS
void Read_Buttons(void);

// MOVE SUB
void Draw_Subs(void);
void Draw_Data(void);
void Get_Data(void);
void Draw_Position(void);

// ESP32
void Read_ESP32(void);

// HELPERS
void printFloatXX(float val);
void computeTilt(int16_t ax, int16_t ay, int16_t az, Tilt_t *t);

static float safe_float(float v) {
    if (!isfinite(v)) return 0.0f;
    return v;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void ST7789_DrawThickLine5(uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1,
                           uint16_t color);

void ST7789_Draw3DAxes_C(uint16_t x0, uint16_t y0);

void ST7789_Draw3DVectorThick5(uint16_t x0, uint16_t y0,
                               float ux, float uy, float uz,
                               uint16_t color);

// PERIODIC
void Get_Joystick(void);

// INTERRUPTS
void UART4_Handler(void);
void GPIOE_Handler(void);
