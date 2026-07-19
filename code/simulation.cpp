#include "esp_heap_caps.h"
#include "esp_log.h"
#include "simulation.h"
#include "esp_attr.h"



// const float Du = 0.16;
// const float Dv = 0.04;
const float Du = 0.1;
const float Dv = 0.05;
extern float F;
extern float k;
const float dt = 1.0;

static float k_norm = 0.059;

// extern float **u;
// static float **v;
// static float **next_u;
// static float **next_v;

extern int rad;
extern volatile float    ui_level;
extern volatile float    ui_freq1;

static float buf_u[2][WIDTH][HEIGHT];
static float buf_v[2][WIDTH][HEIGHT];
float (*u)[HEIGHT]      = buf_u[0];
static float (*v)[HEIGHT]      = buf_v[0];
static float (*next_u)[HEIGHT] = buf_u[1];
static float (*next_v)[HEIGHT] = buf_v[1];
static const char *TAG = "sim";
float threshold = 0.05f;
int note_pos = 0;
void init_grid() {
    ESP_LOGI(TAG, "initializing arrays");
// 	u = (float **)heap_caps_malloc(WIDTH * sizeof(float *), MALLOC_CAP_INTERNAL);
// 	v = (float **)heap_caps_malloc(WIDTH * sizeof(float *), MALLOC_CAP_INTERNAL);
// 	next_u = (float **)heap_caps_malloc(WIDTH * sizeof(float *), MALLOC_CAP_INTERNAL);
// 	next_v = (float **)heap_caps_malloc(WIDTH * sizeof(float *), MALLOC_CAP_INTERNAL);
// 	for (int i = 0; i < WIDTH; i++) {
// 		u[i] = (float *)heap_caps_malloc(WIDTH * sizeof(float), MALLOC_CAP_INTERNAL);
// 		v[i] = (float *)heap_caps_malloc(WIDTH * sizeof(float), MALLOC_CAP_INTERNAL);
// 		next_u[i] = (float *)heap_caps_malloc(WIDTH * sizeof(float), MALLOC_CAP_INTERNAL);
// 		next_v[i] = (float *)heap_caps_malloc(WIDTH * sizeof(float), MALLOC_CAP_INTERNAL);
// 	}
// 	if (!u || !v || !next_u || !next_v) {
// 		ESP_LOGE(TAG, "alloc fail");
// 	}
    for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HEIGHT; y++) {
            // Background state
            u[x][y] = 1.0f;
            v[x][y] = 0.0f;
            
            if (x > WIDTH/2 - rad && x < WIDTH/2 + rad &&
                y > HEIGHT/2 - rad && y < HEIGHT/2 + rad) {
                v[x][y] = 0.4f + 0.02f * ((float)rand() / RAND_MAX - 0.5f);
            }
        }
    }
}



// Fast PRNG — rand() + float division is slow
static inline uint32_t fast_rand(void) {
    static uint32_t s = 0x12345678u;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}

void IRAM_ATTR update_grid(void) {
    if (ui_level > 0.5f) {
        int x_pos = (int)(fast_rand() * (WIDTH * (1.0f / 4294967296.0f)));
        int y_pos = (int)((ui_level * 2.0f - 1.0f) * WIDTH);
        float val = ui_level * 0.3f;                  // multiply, never divide

        // Clamp the rectangle ONCE instead of testing every pixel
        int x0 = x_pos - rad; if (x0 < 0) x0 = 0;
        int x1 = x_pos + rad; if (x1 > WIDTH)  x1 = WIDTH;
        int y0 = y_pos - rad; if (y0 < 0) y0 = 0;
        int y1 = y_pos + rad; if (y1 > HEIGHT) y1 = HEIGHT;

        for (int x = x0; x < x1; x++)
            for (int y = y0; y < y1; y++)
                v[x][y] = val;
    }
    bool wiggle = false;
	if (F == 0.073f && ui_level > threshold){
	F -= 0.015f;
	wiggle = true;
	}
	if (F == 0.040f && ui_level > threshold){
	k += 0.0022f;
	wiggle = true;
	
	}

    // Hoist constants (declare Du, Dv, F, k, dt as float everywhere!)
    const float Fk = F + k;
    for (int x = 0; x < WIDTH; x++) {
        // Resolve x-wrap ONCE per row via row pointers — no modulo in inner loop
        const float *uc = u[x];
        const float *vc = v[x];
        const float *ul = u[(x == 0) ? WIDTH - 1 : x - 1];
        const float *ur = u[(x == WIDTH - 1) ? 0 : x + 1];
        const float *vl = v[(x == 0) ? WIDTH - 1 : x - 1];
        const float *vr = v[(x == WIDTH - 1) ? 0 : x + 1];
        float *nu = next_u[x];
        float *nv = next_v[x];

        // Handle y-wrap by peeling edges — interior loop is 100% branch-free
        // ---- y = 0 ----
        {
            float uxy = uc[0], vxy = vc[0];
            float lap_u = ul[0] + ur[0] + uc[HEIGHT - 1] + uc[1] - 4.0f * uxy;
            float lap_v = vl[0] + vr[0] + vc[HEIGHT - 1] + vc[1] - 4.0f * vxy;
            float uv2 = uxy * vxy * vxy;
            nu[0] = uxy + (Du * lap_u - uv2 + F * (1.0f - uxy)) * dt;
            nv[0] = vxy + (Dv * lap_v + uv2 - Fk * vxy) * dt;
        }
        // ---- interior: sequential access, no branches, no modulo ----
        for (int y = 1; y < HEIGHT - 1; y++) {
            float uxy = uc[y], vxy = vc[y];
            float lap_u = ul[y] + ur[y] + uc[y - 1] + uc[y + 1] - 4.0f * uxy;
            float lap_v = vl[y] + vr[y] + vc[y - 1] + vc[y + 1] - 4.0f * vxy;
            float uv2 = uxy * vxy * vxy;
            if (wiggle && x > note_pos-10 && x < note_pos+10 && y > note_pos-10 && y < note_pos+10) {
				nu[y] = uxy + (Du * lap_u - uv2 + 0.039f * (1.0f - uxy)) * dt;
				nv[y] = vxy + (Dv * lap_v + uv2 - 0.097f * vxy) * dt;
            } else {
            nu[y] = uxy + (Du * lap_u - uv2 + F * (1.0f - uxy)) * dt;
            nv[y] = vxy + (Dv * lap_v + uv2 - Fk * vxy) * dt;}
        }
        // ---- y = HEIGHT-1 ----
        {
            const int y = HEIGHT - 1;
            float uxy = uc[y], vxy = vc[y];
            float lap_u = ul[y] + ur[y] + uc[y - 1] + uc[0] - 4.0f * uxy;
            float lap_v = vl[y] + vr[y] + vc[y - 1] + vc[0] - 4.0f * vxy;
            float uv2 = uxy * vxy * vxy;
            nu[y] = uxy + (Du * lap_u - uv2 + F * (1.0f - uxy)) * dt;
            nv[y] = vxy + (Dv * lap_v + uv2 - Fk * vxy) * dt;
        }
    }

    // Swap buffers — O(1) instead of copying the whole grid
    float (*tmp)[HEIGHT];
    tmp = u;      u = next_u;      next_u = tmp;
    tmp = v;      v = next_v;      next_v = tmp;
    if (wiggle){
    	if (F == 0.073f) F += 0.015f;
    	if (F == 0.040f) k -= 0.0022f;
    }
}
// void update_grid() {
// 	if (ui_level > 0.3) { 
// 		int x_pos = (int)(((float)rand() / RAND_MAX) * WIDTH);
// 		int y_pos = (int)((ui_level * 2.0 - 1) * WIDTH);
// 		for (int x = x_pos - rad; x < x_pos + rad; x++) {
// 			for (int y = y_pos - rad; y < y_pos + rad; y++) {
// 				if (x >= 0 && x < WIDTH && y >= 0 && y < WIDTH) {
// 					v[x][y] = ui_level / 2.0;
// 				}
// 			}
// 		}
// 	}
//     for (int x = 0; x < WIDTH; x++) {
//         for (int y = 0; y < HEIGHT; y++) {
// //         	if (x > (int)(ui_level * WIDTH) - rad && x < (int)(ui_level * WIDTH) + rad
// //         	 && y > (int)(ui_level * HEIGHT)- rad && y < (int)(ui_level * HEIGHT) + rad && ui_level > 0.5)
// //         	{
// //     			v[x][y] = 0.4;
// //     		}
//             //  boundary conditions
//             int left  = (x - 1 + WIDTH) % WIDTH;
//             int right = (x + 1) % WIDTH;
//             int up    = (y - 1 + HEIGHT) % HEIGHT;
//             int down  = (y + 1) % HEIGHT;
// 
//             // Discrete Laplacian
//             float lap_u = u[left][y] + u[right][y] + u[x][up] + u[x][down] - 4.0 * u[x][y];
//             float lap_v = v[left][y] + v[right][y] + v[x][up] + v[x][down] - 4.0 * v[x][y];
// 
//             // Gray-Scott equations
//             float uv2 = u[x][y] * v[x][y] * v[x][y];
//             next_u[x][y] = u[x][y] + (Du * lap_u - uv2 + F * (1.0 - u[x][y])) * dt;
//             next_v[x][y] = v[x][y] + (Dv * lap_v + uv2 - (F + k) * v[x][y]) * dt;
//         }
//     }
// 
//     for (int x = 0; x < WIDTH; x++) {
//         for (int y = 0; y < HEIGHT; y++) {
//             u[x][y] = next_u[x][y];
//             v[x][y] = next_v[x][y];
//         }
//     }
// }

void gs_free() {
	for (int i = 0; i < WIDTH; i++) {
		free(u[i]);
		free(v[i]);
		free(next_u[i]);
		free(next_v[i]);
	}
	free(u);
	free(v);
	free(next_u);
	free(next_v);
}

