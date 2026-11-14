#include "board.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "input";

/* Gesture tuning.  Long = PTT, so the threshold is the safety margin between
 * "I'm tuning the knob" and "I'm on the air". */
#define LONG_MS        600
#define DOUBLE_GAP_MS  350
#define REPEAT_DELAY_MS 450
#define REPEAT_PERIOD_MS 120
#define TICK_MS        10

static QueueHandle_t s_q;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void push(input_event_t ev)
{
    if (!s_q) return;
    /* Never block the input task; a dropped repeat is harmless. */
    xQueueSend(s_q, &ev, 0);
}

input_event_t board_input_get_event(void)
{
    input_event_t ev = INPUT_NONE;
    if (s_q) xQueueReceive(s_q, &ev, 0);
    return ev;
}

static void gpio_setup(gpio_num_t pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

typedef struct {
    bool    down;        /* debounced */
    bool    raw;
    int     stable;
    int64_t t_down;
    int64_t t_last_rep;
    bool    rep_fired;
} key_t;

static bool key_poll(key_t *k, gpio_num_t pin, int64_t t)
{
    bool raw = (gpio_get_level(pin) == 0);      /* active low */
    if (raw != k->raw) {
        k->raw = raw;
        k->stable = 0;
        return false;
    }
    if (k->stable < 3) { k->stable++; return false; }
    if (raw != k->down) {
        k->down = raw;
        if (raw) {
            k->t_down = t;
            k->rep_fired = false;
        }
        return true;                             /* edge */
    }
    return false;
}

static void input_task(void *arg)
{
    gpio_setup(PIN_BTN_BOOT);
    gpio_setup(PIN_BTN_VOL_UP);
    gpio_setup(PIN_BTN_VOL_DN);

    key_t boot = {0}, up = {0}, dn = {0};

    enum { BS_IDLE, BS_PRESS, BS_HELD, BS_WAIT2, BS_PRESS2 } bs = BS_IDLE;
    int64_t t_state = now_ms();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        int64_t t = now_ms();

        /* ── BOOT: short/short = toggle, long = PTT ─────────────── */
        bool boot_edge = key_poll(&boot, PIN_BTN_BOOT, t);
        switch (bs) {
        case BS_IDLE:
            if (boot_edge && boot.down) { bs = BS_PRESS; t_state = t; }
            break;
        case BS_PRESS:
            if (boot_edge && !boot.down) { bs = BS_WAIT2; t_state = t; }
            else if (boot.down && t - t_state >= LONG_MS) {
                push(INPUT_BOOT_LONG);
                bs = BS_HELD;
            }
            break;
        case BS_HELD:
            if (boot_edge && !boot.down) { push(INPUT_BOOT_RELEASE); bs = BS_IDLE; }
            break;
        case BS_WAIT2:
            if (boot_edge && boot.down) { bs = BS_PRESS2; t_state = t; }
            else if (!boot.down && t - t_state > DOUBLE_GAP_MS) bs = BS_IDLE;
            break;
        case BS_PRESS2:
            if (boot_edge && !boot.down) { push(INPUT_BOOT_DOUBLE); bs = BS_IDLE; }
            else if (boot.down && t - t_state >= LONG_MS) {
                push(INPUT_BOOT_LONG);
                bs = BS_HELD;
            }
            break;
        }

        /* ── VOL+/VOL-: click + auto-repeat ─────────────────────── */
        bool up_edge = key_poll(&up, PIN_BTN_VOL_UP, t);
        if (up_edge && up.down) push(INPUT_VOL_UP);
        if (up.down && t - up.t_down >= REPEAT_DELAY_MS &&
            t - up.t_last_rep >= REPEAT_PERIOD_MS) {
            up.t_last_rep = t;
            push(INPUT_VOL_UP);
        }

        bool dn_edge = key_poll(&dn, PIN_BTN_VOL_DN, t);
        if (dn_edge && dn.down) push(INPUT_VOL_DOWN);
        if (dn.down && t - dn.t_down >= REPEAT_DELAY_MS &&
            t - dn.t_last_rep >= REPEAT_PERIOD_MS) {
            dn.t_last_rep = t;
            push(INPUT_VOL_DOWN);
        }
    }
}

void board_input_init(void)
{
    s_q = xQueueCreate(16, sizeof(input_event_t));
    xTaskCreatePinnedToCore(input_task, "btn", 3072, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "buttons: BOOT gpio%d (long=PTT, double=FREQ/VOL), "
             "VOL+ gpio%d, VOL- gpio%d",
             PIN_BTN_BOOT, PIN_BTN_VOL_UP, PIN_BTN_VOL_DN);
}

/* ── Battery / charging ──────────────────────────────────────────── */

static adc_oneshot_unit_handle_t s_adc;
static bool s_adc_tried;

void board_power_status(int *battery_percent, bool *charging)
{
    if (charging) *charging = gpio_get_level(PIN_CHARGE_DET) == 1;
    if (!battery_percent) return;

    *battery_percent = -1;
    if (!s_adc_tried) {
        s_adc_tried = true;
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << PIN_CHARGE_DET,
            .mode         = GPIO_MODE_INPUT,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        adc_oneshot_unit_init_cfg_t init = {
            .unit_id  = ADC_UNIT_2,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        if (adc_oneshot_new_unit(&init, &s_adc) != ESP_OK) s_adc = NULL;
        if (s_adc) {
            adc_oneshot_chan_cfg_t ch = {
                .atten    = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_12,
            };
            adc_oneshot_config_channel(s_adc, ADC_CHANNEL_6, &ch);
        }
    }
    if (!s_adc) return;

    /* ADC2 shares hardware with the radio; while Wi-Fi is busy the read can
     * fail.  Retry a few times and give up quietly — the UI shows "--". */
    int raw = 0;
    for (int i = 0; i < 3; i++) {
        if (adc_oneshot_read(s_adc, ADC_CHANNEL_6, &raw) == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(2));
        raw = -1;
    }
    if (raw < 0) return;

    static const struct { int adc; int pct; } levels[] = {
        {1970, 0}, {2062, 20}, {2154, 40}, {2246, 60}, {2338, 80}, {2430, 100},
    };
    if (raw < levels[0].adc) { *battery_percent = 0; return; }
    if (raw >= levels[5].adc) { *battery_percent = 100; return; }
    for (int i = 0; i < 5; i++) {
        if (raw >= levels[i].adc && raw < levels[i + 1].adc) {
            int span = levels[i + 1].adc - levels[i].adc;
            *battery_percent = levels[i].pct +
                (raw - levels[i].adc) * (levels[i + 1].pct - levels[i].pct) / span;
            return;
        }
    }
}
