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

    int frameNum = 0;
    double totalAdvanceTime = 0.0;
    while (!g_window->windowClosed && !g_window->input.keys[KEY_ESC]) {
        BitmapClear(g_window->bmp, g_colourBlack);
        InputPoll(g_window);

        double startTime = GetRealTime();
        g_world.Advance();
        double duration = GetRealTime() - startTime;
        if (frameNum < 500)
            totalAdvanceTime += duration;

        g_world.Render(g_window->bmp);

        RectFill(g_window->bmp, g_window->bmp->width - 54, 0, 54, 13, g_colourBlack);
        DrawTextRight(font, g_colourWhite, g_window->bmp, g_window->bmp->width - 2, 0, "FPS:%i", g_window->fps);

        RectFill(g_window->bmp, 0, 0, 130, 13, g_colourBlack);
        DrawTextLeft(font, g_colourWhite, g_window->bmp, 2, 0, "Bench Time: %.2f", totalAdvanceTime);

        UpdateWin(g_window);

        frameNum++;
    }

    return 0;
}
