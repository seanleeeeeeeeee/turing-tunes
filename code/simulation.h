#pragma once

#define WIDTH 100
#define HEIGHT 100
#define STEPS 100
extern float (*u)[HEIGHT];
extern float threshold;
extern int note_pos;
void init_grid();
void update_grid();
void gs_free();