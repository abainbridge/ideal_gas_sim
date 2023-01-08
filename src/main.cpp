// Project headers
#include "world.h"

// Deadfrog headers
#include "df_bmp.h"
#include "df_font.h"
#include "fonts/df_mono.h"
#include "df_time.h"
#include "df_window.h"

// Standard headers
#include <math.h>
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>


int main(int argc, char *argv[]) {
    g_window = CreateWin(WORLD_SIZE_X * 1.5, WORLD_SIZE_Y * 1.5, WT_WINDOWED_FIXED, "Ideal Gas Simulator");
    g_world.m_viewScale = (float)g_window->bmp->width / WORLD_SIZE_X;
    DfFont *font = LoadFontFromMemory(df_mono_8x15, sizeof(df_mono_8x15));

    double startTime = GetRealTime();
    double duration = 0.0;
    unsigned frameNum = 0;
    while (!g_window->windowClosed && !g_window->input.keys[KEY_ESC]) {
        BitmapClear(g_window->bmp, g_colourBlack);
        InputPoll(g_window);

        g_world.Advance();
        g_world.Render(g_window->bmp);

        DrawTextRight(font, g_colourWhite, g_window->bmp, g_window->bmp->width - 5, 0, "FPS:%i", g_window->fps);
        DrawTextLeft(font, g_colourWhite, g_window->bmp, 0, 0, "Bench Time:%.2f", duration);

//         if (frameNum == 500)
//         {
//             // We got to the end of the benchmark run, stop the clock!
//             double endTime = DfGetTime();
//             duration = endTime - startTime;
//         }

        UpdateWin(g_window);
//        DfSleepMillisec(10);

        frameNum++;
    }

    return 0;
}
