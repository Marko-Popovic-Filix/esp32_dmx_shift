#include "dmx_ctrl.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "cJSON.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DMX_UART            UART_NUM_1
#define DMX_TX_PIN          GPIO_NUM_14
#define DMX_DE_PIN          GPIO_NUM_4
#define DMX_UNIVERSE_SIZE   512
#define DMX_FPS             40

#define BREATHE_PERIOD_MS   3000

// --- Strobe params ---
#define STROBE_HZ           8.0f
#define STROBE_DUTY         0.50f

// --- SOS (Morse) params ---
#define SOS_UNIT_MS         120

// --- Flash-3 (broadcast) params ---
#define FLASH3_ON_MS        180.0f   // on-time per flash
#define FLASH3_OFF_MS       180.0f   // off-time per flash
#define FLASH3_COUNT        3        // exactly 3 flashes

static const char *TAG = "dmx_ctrl";

/* Universe buffer with start code at index 0.
 * Channel 1 lives at dmx_buffer[1]. */
static uint8_t dmx_buffer[DMX_UNIVERSE_SIZE + 1];
static TaskHandle_t dmx_task_handle = NULL;

typedef enum { MODE_OFF=0, MODE_STATIC, MODE_BREATHE, MODE_STROBE, MODE_SOS } mode_t;

typedef struct {
    int base;                 // 1-based DMX start channel (R at base, then G,B,W)
    mode_t mode;
    uint8_t static_rgba[4];   // for MODE_STATIC
    uint8_t amp_rgba[4];      // amplitude / peak values for effects
    uint8_t chan_mask[4];     // which channels are active (1) vs OFF (0)
} group_state_t;

/* Fixtures (RGBW layout for all):
 * FARO100-1: CH 1..4
 * FARO100-2: CH 5..8
 * RDD200-1 : CH 9..12
 * RDD200-2 : CH 13..16
 */
static group_state_t g_faro1 = { .base = 1,  .mode = MODE_OFF };
static group_state_t g_faro2 = { .base = 5,  .mode = MODE_OFF };
static group_state_t g_rdd1  = { .base = 9,  .mode = MODE_OFF };
static group_state_t g_rdd2  = { .base = 13, .mode = MODE_OFF };

/* ---- Global “flash 3x” override (broadcast) ---- */
typedef struct {
    bool        active;
    TickType_t  t0;          // start tick
    uint8_t     rgba[4];     // levels to flash with
    uint8_t     mask[4];     // which channels flash
} flash3_t;

static flash3_t s_flash3 = {0};

/* ---- DMX send helpers ---- */
static inline void dmx_send_frame(void) {
    // Generate BREAK by inverting TX line low for ~176 us
    uart_set_line_inverse(DMX_UART, UART_SIGNAL_TXD_INV);
    esp_rom_delay_us(176);
    uart_set_line_inverse(DMX_UART, 0);
    // Mark After Break
    esp_rom_delay_us(12);

    uart_write_bytes(DMX_UART, (const char*)dmx_buffer, DMX_UNIVERSE_SIZE + 1);
    uart_wait_tx_done(DMX_UART, 20 / portTICK_PERIOD_MS);
}

static inline uint8_t clamp255i(int v) {
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

/* Waves */
static inline uint8_t breathe_wave(float t_ms, float period_ms, uint8_t amp) {
    float y = (1.0f - cosf(2.0f * (float)M_PI * t_ms / period_ms)) * 0.5f; // 0..1
    return clamp255i((int)lrintf(y * (float)amp));
}

static inline uint8_t strobe_wave(float t_ms, float hz, float duty, uint8_t amp) {
    if (hz <= 0.0f) return 0;
    float period_ms = 1000.0f / hz;
    float phase = fmodf(t_ms, period_ms) / period_ms; // 0..1
    return (phase < duty) ? amp : 0;
}

/* SOS pattern in plain C (… --- …) */
static bool sos_on_at(float t_ms) {
    typedef struct { uint8_t on_u; uint8_t off_u; } seg_t;
    static const seg_t segs[] = {
        // S
        {1,1}, {1,1}, {1,3},
        // O
        {3,1}, {3,1}, {3,3},
        // S
        {1,1}, {1,1}, {1,7},
    };
    static const int seg_count = (int)(sizeof(segs)/sizeof(segs[0]));

    int total_u = 0;
    for (int i=0;i<seg_count;i++) total_u += segs[i].on_u + segs[i].off_u;
    if (total_u <= 0) total_u = 1;

    float unit_ms = (float)SOS_UNIT_MS;
    float T_ms = unit_ms * (float)total_u;
    float tm = fmodf(t_ms, T_ms);
    float acc = 0.0f;

    for (int i=0;i<seg_count;i++) {
        float on_ms  = (float)segs[i].on_u  * unit_ms;
        float off_ms = (float)segs[i].off_u * unit_ms;

        if (on_ms > 0.0f) {
            if (tm >= acc && tm < acc + on_ms) return true;
            acc += on_ms;
        }
        if (off_ms > 0.0f) {
            if (tm >= acc && tm < acc + off_ms) return false;
            acc += off_ms;
        }
    }
    return false;
}

/* Render helpers */
static void render_group(group_state_t *g, float t_ms){
    // Clear this group's slots first
    for (int i = 0; i < 4; i++){
        int ch = g->base + i;
        if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE) dmx_buffer[ch] = 0;
    }

    switch (g->mode){
        case MODE_OFF:
            break;

        case MODE_STATIC:
            for (int i = 0; i < 4; i++){
                int ch = g->base + i;
                if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE)
                    dmx_buffer[ch] = g->static_rgba[i];
            }
            break;

        case MODE_BREATHE:
            for (int i = 0; i < 4; i++){
                int ch = g->base + i;
                if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE){
                    dmx_buffer[ch] = g->chan_mask[i]
                        ? breathe_wave(t_ms, (float)BREATHE_PERIOD_MS, g->amp_rgba[i])
                        : 0;
                }
            }
            break;

        case MODE_STROBE:
            for (int i = 0; i < 4; i++){
                int ch = g->base + i;
                if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE){
                    dmx_buffer[ch] = g->chan_mask[i]
                        ? strobe_wave(t_ms, STROBE_HZ, STROBE_DUTY, g->amp_rgba[i])
                        : 0;
                }
            }
            break;

        case MODE_SOS: {
            bool on = sos_on_at(t_ms);
            for (int i = 0; i < 4; i++){
                int ch = g->base + i;
                if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE){
                    dmx_buffer[ch] = (g->chan_mask[i] && on) ? g->amp_rgba[i] : 0;
                }
            }
        }   break;
    }
}

static void render_flash3_all(float t_ms){
    float period = FLASH3_ON_MS + FLASH3_OFF_MS;
    float elapsed = (float)((xTaskGetTickCount() - s_flash3.t0) * portTICK_PERIOD_MS);

    // Stop after 3 complete flashes
    if (elapsed >= (float)FLASH3_COUNT * period) {
        s_flash3.active = false;
        return;
    }

    float phase = fmodf(elapsed, period);
    bool on = (phase < FLASH3_ON_MS);

    // Apply to all 4 fixtures
    group_state_t* gs[] = { &g_faro1, &g_faro2, &g_rdd1, &g_rdd2 };
    for (int g = 0; g < 4; g++) {
        for (int i = 0; i < 4; i++){
            int ch = gs[g]->base + i;
            if (ch >= 1 && ch <= DMX_UNIVERSE_SIZE){
                dmx_buffer[ch] = (on && s_flash3.mask[i]) ? s_flash3.rgba[i] : 0;
            }
        }
    }
}

static void dmx_task(void *arg) {
    ESP_LOGI(TAG, "DMX task started on core %d", xPortGetCoreID());
    TickType_t t0 = xTaskGetTickCount();
    const TickType_t dt = pdMS_TO_TICKS(1000 / DMX_FPS);

    while (1) {
        float t_ms = (float)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);

        dmx_buffer[0] = 0x00; // start code

        if (s_flash3.active) {
            // Broadcast flash overrides everything while active
            render_flash3_all(t_ms);
        } else {
            render_group(&g_faro1, t_ms);
            render_group(&g_faro2, t_ms);
            render_group(&g_rdd1,  t_ms);
            render_group(&g_rdd2,  t_ms);
        }

        dmx_send_frame();
        vTaskDelay(dt);
    }
}

/* ---- Public API ---- */
esp_err_t dmx_ctrl_start_on_core(BaseType_t core_id) {
    uart_config_t uc = {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_REF_TICK,
    };
    ESP_ERROR_CHECK(uart_param_config(DMX_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(DMX_UART, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // TX-only; minimal RX buffer avoids driver error
    const int RX_BUF = 256;
    const int TX_BUF = 0;
    ESP_ERROR_CHECK(uart_driver_install(DMX_UART, RX_BUF, TX_BUF, 0, NULL, 0));

    // Enable RS485 driver (DE high)
    gpio_set_direction(DMX_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DMX_DE_PIN, 1);

    memset(dmx_buffer, 0, sizeof(dmx_buffer));
    dmx_buffer[0] = 0x00; // start code

    if (dmx_task_handle) vTaskDelete(dmx_task_handle);
    xTaskCreatePinnedToCore(dmx_task, "dmx_task", 4096, NULL, 5, &dmx_task_handle, core_id);
    return ESP_OK;
}

void dmx_ctrl_stop(void) {
    if (dmx_task_handle) {
        vTaskDelete(dmx_task_handle);
        dmx_task_handle = NULL;
    }
    uart_driver_delete(DMX_UART);
}

/* Map deviceId -> group */
static group_state_t* pick_group(const char *id){
    if (!id) return NULL;
    if (strcasecmp(id, "FARO100-1") == 0) return &g_faro1;
    if (strcasecmp(id, "FARO100-2") == 0) return &g_faro2;
    if (strcasecmp(id, "RDD200-1")  == 0) return &g_rdd1;
    if (strcasecmp(id, "RDD200-2")  == 0) return &g_rdd2;
    return NULL;
}

/* Setters */
static void set_off(group_state_t *g){
    g->mode = MODE_OFF;
    memset(g->static_rgba, 0, 4);
    memset(g->amp_rgba,    0, 4);
    memset(g->chan_mask,   0, 4);
}

static void set_static(group_state_t *g, int R,int G,int B,int W){
    g->mode = MODE_STATIC;
    g->static_rgba[0] = (uint8_t)clamp255i(R);
    g->static_rgba[1] = (uint8_t)clamp255i(G);
    g->static_rgba[2] = (uint8_t)clamp255i(B);
    g->static_rgba[3] = (uint8_t)clamp255i(W);
    memset(g->amp_rgba,  0, 4);
    memset(g->chan_mask, 0, 4);
}

static void set_breathe(group_state_t *g, int R,int G,int B,int W, int mR,int mG,int mB,int mW){
    g->mode = MODE_BREATHE;
    g->amp_rgba[0] = (uint8_t)clamp255i(R);
    g->amp_rgba[1] = (uint8_t)clamp255i(G);
    g->amp_rgba[2] = (uint8_t)clamp255i(B);
    g->amp_rgba[3] = (uint8_t)clamp255i(W);
    g->chan_mask[0] = mR?1:0;
    g->chan_mask[1] = mG?1:0;
    g->chan_mask[2] = mB?1:0;
    g->chan_mask[3] = mW?1:0;
    memset(g->static_rgba, 0, 4);
}

static void set_strobe(group_state_t *g, int R,int G,int B,int W, int mR,int mG,int mB,int mW){
    g->mode = MODE_STROBE;
    g->amp_rgba[0] = (uint8_t)clamp255i(R);
    g->amp_rgba[1] = (uint8_t)clamp255i(G);
    g->amp_rgba[2] = (uint8_t)clamp255i(B);
    g->amp_rgba[3] = (uint8_t)clamp255i(W);
    g->chan_mask[0] = mR?1:0;
    g->chan_mask[1] = mG?1:0;
    g->chan_mask[2] = mB?1:0;
    g->chan_mask[3] = mW?1:0;
    memset(g->static_rgba, 0, 4);
}

static void set_sos(group_state_t *g, int R,int G,int B,int W, int mR,int mG,int mB,int mW){
    g->mode = MODE_SOS;
    g->amp_rgba[0] = (uint8_t)clamp255i(R);
    g->amp_rgba[1] = (uint8_t)clamp255i(G);
    g->amp_rgba[2] = (uint8_t)clamp255i(B);
    g->amp_rgba[3] = (uint8_t)clamp255i(W);
    g->chan_mask[0] = mR?1:0;
    g->chan_mask[1] = mG?1:0;
    g->chan_mask[2] = mB?1:0;
    g->chan_mask[3] = mW?1:0;
    memset(g->static_rgba, 0, 4);
}

/* Command helper */
static void apply_command_to_group(group_state_t *g, const char *c, int R,int G,int B,int W){
    if      (strcasecmp(c, "off") == 0)             set_off(g);
    else if (strcasecmp(c, "on")  == 0)             set_static(g, R,G,B,W);

    else if (strcasecmp(c, "breathe all") == 0)     set_breathe(g, R,G,B,W, 1,1,1,1);
    else if (strcasecmp(c, "breathe r")   == 0)     set_breathe(g, R,G,B,W, 1,0,0,0);
    else if (strcasecmp(c, "breathe g")   == 0)     set_breathe(g, R,G,B,W, 0,1,0,0);
    else if (strcasecmp(c, "breathe b")   == 0)     set_breathe(g, R,G,B,W, 0,0,1,0);
    else if (strcasecmp(c, "breathe w")   == 0)     set_breathe(g, R,G,B,W, 0,0,0,1);

    else if (strcasecmp(c, "strobe all")  == 0)     set_strobe(g,  R,G,B,W, 1,1,1,1);
    else if (strcasecmp(c, "strobe r")    == 0)     set_strobe(g,  R,G,B,W, 1,0,0,0);
    else if (strcasecmp(c, "strobe g")    == 0)     set_strobe(g,  R,G,B,W, 0,1,0,0);
    else if (strcasecmp(c, "strobe b")    == 0)     set_strobe(g,  R,G,B,W, 0,0,1,0);
    else if (strcasecmp(c, "strobe w")    == 0)     set_strobe(g,  R,G,B,W, 0,0,0,1);

    else if (strcasecmp(c, "sos")         == 0)     set_sos(g,     R,G,B,W, 0,0,0,1);
    else if (strcasecmp(c, "sos all")     == 0)     set_sos(g,     R,G,B,W, 1,1,1,1);
    else if (strcasecmp(c, "sos r")       == 0)     set_sos(g,     R,G,B,W, 1,0,0,0);
    else if (strcasecmp(c, "sos g")       == 0)     set_sos(g,     R,G,B,W, 0,1,0,0);
    else if (strcasecmp(c, "sos b")       == 0)     set_sos(g,     R,G,B,W, 0,0,1,0);
    else if (strcasecmp(c, "sos w")       == 0)     set_sos(g,     R,G,B,W, 0,0,0,1);

    else                                            set_static(g, R,G,B,W);
}

/* Trigger 3x broadcast flash */
static void flash3_start_rgbw(uint8_t R,uint8_t G,uint8_t B,uint8_t W, uint8_t mR,uint8_t mG,uint8_t mB,uint8_t mW){
    s_flash3.active = true;
    s_flash3.t0     = xTaskGetTickCount();
    s_flash3.rgba[0]= R; s_flash3.rgba[1]= G; s_flash3.rgba[2]= B; s_flash3.rgba[3]= W;
    s_flash3.mask[0]= mR?1:0; s_flash3.mask[1]= mG?1:0; s_flash3.mask[2]= mB?1:0; s_flash3.mask[3]= mW?1:0;
}

esp_err_t dmx_ctrl_apply_json(const char *json, int len){
    if (!json || len <= 0) return ESP_ERR_INVALID_ARG;
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) return ESP_ERR_INVALID_ARG;

    const cJSON *dev = cJSON_GetObjectItem(root, "deviceId");
    const cJSON *cmd = cJSON_GetObjectItem(root, "command");
    const cJSON *jR  = cJSON_GetObjectItem(root, "R");
    const cJSON *jG  = cJSON_GetObjectItem(root, "G");
    const cJSON *jB  = cJSON_GetObjectItem(root, "B");
    const cJSON *jW  = cJSON_GetObjectItem(root, "W");

    const char *dev_id = cJSON_IsString(dev) ? dev->valuestring : NULL;
    int R = cJSON_IsNumber(jR) ? jR->valueint : 0;
    int G = cJSON_IsNumber(jG) ? jG->valueint : 0;
    int B = cJSON_IsNumber(jB) ? jB->valueint : 0;
    int W = cJSON_IsNumber(jW) ? jW->valueint : 0;

    if (cJSON_IsString(cmd) && cmd->valuestring){
        const char *c = cmd->valuestring;

        // NEW: boolean-style broadcast flashes
        if (strcasecmp(c, "true") == 0) {
            // flash GREEN 3x on all devices
            flash3_start_rgbw(0, 255, 0, 0, 0,1,0,0);
        } else if (strcasecmp(c, "false") == 0) {
            // flash RED 3x on all devices
            flash3_start_rgbw(255, 0, 0, 0, 1,0,0,0);
        } else {
            // normal device/broadcast control
            if (!dev_id || strcasecmp(dev_id, "all") == 0) {
                apply_command_to_group(&g_faro1, c, R,G,B,W);
                apply_command_to_group(&g_faro2, c, R,G,B,W);
                apply_command_to_group(&g_rdd1,  c, R,G,B,W);
                apply_command_to_group(&g_rdd2,  c, R,G,B,W);
            } else {
                group_state_t *g = pick_group(dev_id);
                if (g) apply_command_to_group(g, c, R,G,B,W);
            }
        }
    } else {
        // No command: treat as static set
        if (!dev_id || strcasecmp(dev_id, "all") == 0) {
            set_static(&g_faro1, R,G,B,W);
            set_static(&g_faro2, R,G,B,W);
            set_static(&g_rdd1,  R,G,B,W);
            set_static(&g_rdd2,  R,G,B,W);
        } else {
            group_state_t *g = pick_group(dev_id);
            if (g) set_static(g, R,G,B,W);
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}
