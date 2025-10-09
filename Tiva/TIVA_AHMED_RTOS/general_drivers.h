
#pragma once

/************************************Includes***************************************/

//#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"
//#include "final_threads.h"    
//#include <time.h>

/************************************MAIN*******************************************/

#include <stdint.h>
#include <stdlib.h> // for abs()

#define WIDTH  120
#define HEIGHT 160

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)((((r >> 3) & 0x1F) << 11) | (((g >> 2) & 0x3F) << 5) | ((b >> 3) & 0x1F));
}

/* Geometry constants (scaled) */
static const int SUB_CX = 70;
static const int SUB_CY = 80;
static const int BODY_RX = 30;
static const int BODY_RY = 18;

/* Core generator used by both wrappers.
   draw_x = 0 -> normal image
   draw_x = 1 -> draw big red X overlay on top */
static uint16_t get_submarine_pixel_core(int x, int y, int draw_x)
{
    if ((unsigned)x >= (unsigned)WIDTH || (unsigned)y >= (unsigned)HEIGHT) return 0;

    // Colors
    const uint16_t green_bg         = 0xEFBF;    // green background
    const uint16_t light_green      = rgb565(80, 200, 80);
    const uint16_t sub_gray         = rgb565(80, 80, 80);
    const uint16_t sub_dark         = rgb565(40, 40, 40);
    const uint16_t window_color     = rgb565(200, 200, 220);
    const uint16_t sonar_red        = ST7789_RED;  // red for detection and X and frame
    const uint16_t highlight_yellow = rgb565(255, 220, 0);

    uint16_t color = green_bg; // background

    // --- sonar rings (behind sub)
    {
        const int sonar_cx = SUB_CX;
        const int sonar_cy = SUB_CY - 6;
        long dx = x - sonar_cx;
        long dy = y - sonar_cy;
        long d2 = dx*dx + dy*dy;
        const int radii[] = {20, 30, 40};
        for (size_t i = 0; i < sizeof(radii)/sizeof(radii[0]); ++i) {
            int r = radii[i];
            long r2 = (long)r * r;
            long inner = (long)(r - 1) * (r - 1);
            if (d2 <= r2 && d2 >= inner) { color = light_green; break; }
        }
    }

    // --- submarine hull & shading
    {
        // main body ellipse
        long dx = x - SUB_CX;
        long dy = y - SUB_CY;
        long rx2 = (long)BODY_RX * BODY_RX;
        long ry2 = (long)BODY_RY * BODY_RY;
        if ((dx*dx) * ry2 + (dy*dy) * rx2 <= rx2 * ry2) color = sub_gray;

        // shading (lower part)
        const int scy = SUB_CY + 3;
        const int srx = BODY_RX - 3;
        const int sry = BODY_RY - 3;
        if (srx > 0 && sry > 0) {
            long dy2 = y - scy;
            long rx2s = (long)srx * srx;
            long ry2s = (long)sry * sry;
            if ((dx*dx) * ry2s + (dy2*dy2) * rx2s <= rx2s * ry2s) color = sub_dark;
        }
    }

    // --- tail (rect + triangular fin)
    {
        int tx0 = SUB_CX + BODY_RX - 3;
        int ty0 = SUB_CY - 6;
        int tw  = 15, th = 12;
        if (x >= tx0 && x < tx0 + tw && y >= ty0 && y < ty0 + th) {
            color = sub_dark;
        } else {
            int tri_x0 = SUB_CX + BODY_RX + 14;
            int tri_x1 = SUB_CX + BODY_RX + 26;
            if (x >= tri_x0 && x <= tri_x1 && y >= SUB_CY - 12 && y <= SUB_CY + 12) {
                int dx = x - tri_x0;
                if (dx <= abs(y - SUB_CY)) color = sub_dark;
            }
        }
    }

    // --- conning tower & periscope
    {
        int tx = SUB_CX - 5, ty = SUB_CY - BODY_RY - 5;
        if (x >= tx && x < tx + 13 && y >= ty && y < ty + 11) color = sub_gray;
        int px = SUB_CX + 3, py = SUB_CY - BODY_RY - 13;
        if (x >= px && x < px + 2 && y >= py && y < py + 9) color = sub_dark;
        if (x >= (SUB_CX - 1) && x < (SUB_CX - 1) + 7 && y >= py && y < py + 2) color = sub_dark;
    }

    // --- portholes
    {
        const int r = 3;
        long dy1 = y - SUB_CY;
        long dx1 = x - (SUB_CX - 13);
        if (dx1*dx1 + dy1*dy1 <= (long)r*r) color = window_color;
        long dx2 = x - (SUB_CX - 2);
        if (dx2*dx2 + dy1*dy1 <= (long)r*r) color = window_color;
        long dx3 = x - (SUB_CX + 10);
        if (dx3*dx3 + dy1*dy1 <= (long)r*r) color = window_color;
    }

    // --- small highlight
    {
        long hx = x - (SUB_CX - 9);
        long hy = y - (SUB_CY - 7);
        if (hx*hx + hy*hy <= 45) color = highlight_yellow;
    }

    // --- detection overlay: label bar (red with yellow stripe)
    {
        int lab_x = SUB_CX - 20;
        int lab_y = SUB_CY - BODY_RY - 28;
        int lab_w = 40;
        int lab_h = 10;
        if (x >= lab_x && x < lab_x + lab_w && y >= lab_y && y < lab_y + lab_h) {
            int yx = SUB_CX - 18;
            int yy = SUB_CY - BODY_RY - 26;
            if (x >= yx && x < yx + 36 && y >= yy && y < yy + 4) {
                color = highlight_yellow;
            } else {
                color = sonar_red;
            }
        }
    }

    // detection ring and crosshair (on top)
    {
        const int detect_r = 23;
        long dx = x - SUB_CX;
        long dy = y - SUB_CY;
        long d2 = dx*dx + dy*dy;
        long outer = (long)detect_r * detect_r;
        long inner = (long)(detect_r - 1) * (detect_r - 1);
        if (d2 <= outer && d2 >= inner) color = sonar_red;

        if ((y >= SUB_CY - 1 && y <= SUB_CY + 1) &&
            (x >= SUB_CX - detect_r - 4 && x <= SUB_CX + detect_r + 4)) color = sonar_red;
        if ((x >= SUB_CX - 1 && x <= SUB_CX + 1) &&
            (y >= SUB_CY - detect_r - 4 && y <= SUB_CY + detect_r + 4)) color = sonar_red;
    }

    // --- If drawing the big X requested, draw red X on top (returns to ensure X sits above everything)
    if (draw_x) {
        // bounding rectangle around submarine to limit where X is drawn
        int x0 = SUB_CX - BODY_RX, y0 = SUB_CY - BODY_RY;
        int x1 = SUB_CX + BODY_RX, y1 = SUB_CY + BODY_RY;

        // line TL->BR
        long lx = x1 - x0, ly = y1 - y0;
        long A = ly, B = -lx;
        long C1 = (long)x1*y0 - (long)y1*x0;
        long num1 = A*(long)x + B*(long)y + C1; if (num1 < 0) num1 = -num1;
        long len2_1 = lx*lx + ly*ly;
        const long thick = 3;
        if (num1 * num1 <= (thick*thick) * len2_1) return sonar_red;

        // line TR->BL
        long lx2 = x0 - x1, ly2 = y1 - y0;
        long A2 = ly2, B2 = -lx2;
        long C2 = (long)x0*y0 - (long)y1*x1;
        long num2 = A2*(long)x + B2*(long)y + C2; if (num2 < 0) num2 = -num2;
        long len2_2 = lx2*lx2 + ly2*ly2;
        if (num2 * num2 <= (thick*thick) * len2_2) return sonar_red;
    }

    // --- FRAME: 5-pixel red border around entire image (drawn on top of everything)
    const int frame_thickness = 5;
    if (x < frame_thickness || x >= WIDTH - frame_thickness ||
        y < frame_thickness || y >= HEIGHT - frame_thickness) {
        return sonar_red; // red frame
    }

    return color;
}

/* Public wrappers */
static inline uint16_t get_submarine_pixel(int x, int y) {
    return get_submarine_pixel_core(x, y, 0);
}
static inline uint16_t get_submarine_pixel_with_x(int x, int y) {
    return get_submarine_pixel_core(x, y, 1);
}
