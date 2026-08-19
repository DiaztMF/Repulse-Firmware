#include "Repulse_UI.h"

#include "BAT_Driver.h"
#include "LVGL_Driver.h"
#include "RTC_PCF85063.h"
#include "Repulse_Storage.h"

#include <Preferences.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define COLOR_BASE      lv_color_hex(0x100D0A)
#define COLOR_SURFACE   lv_color_hex(0x191512)
#define COLOR_RAISED    lv_color_hex(0x24201B)
#define COLOR_DIVIDER   lv_color_hex(0x3A332D)
#define COLOR_PRIMARY   lv_color_hex(0xE7E0D7)
#define COLOR_SECONDARY lv_color_hex(0xA69B8F)
#define COLOR_MUTED     lv_color_hex(0x756C63)
#define COLOR_PULSE     lv_color_hex(0xD59A4A)
#define COLOR_SLEEP     lv_color_hex(0xB89072)
#define COLOR_BREATH    lv_color_hex(0xB87666)
#define COLOR_SUCCESS   lv_color_hex(0x7E9C86)
#define COLOR_WARNING   lv_color_hex(0xD0A05C)
#define COLOR_DANGER    lv_color_hex(0xC9575D)
#define COLOR_SOS       lv_color_hex(0xE45B60)
#define COLOR_IVORY     lv_color_hex(0xEDE3D6)

static const int16_t SPLASH_CANVAS_WIDTH = 260;
static const int16_t SPLASH_CANVAS_HEIGHT = 48;
static const int16_t WORDMARK_SOURCE_WIDTH = 582;
static const int16_t WORDMARK_SOURCE_HEIGHT = 104;
static const uint16_t WORDMARK_DRAW_MS = 1100;
static const uint16_t SPLASH_DWELL_MS = 1750;
static const uint8_t DEFAULT_DAY_BRIGHTNESS = 22;
static const uint8_t DEFAULT_NIGHT_BRIGHTNESS = 12;
static const uint8_t IDLE_DIM_BRIGHTNESS = 5;
static const uint8_t ALERT_BRIGHTNESS_MIN = 35;
static const uint16_t DEFAULT_TIMEOUT_SECONDS = 25;

enum DemoAction {
    DEMO_NORMAL,
    DEMO_COMFORT,
    DEMO_OFFLINE,
    DEMO_ALERT,
    DEMO_SOS,
    DEMO_BACK
};

enum EscalationStage {
    ESCALATION_NONE,
    ESCALATION_STAGE_2,
    ESCALATION_STAGE_3,
    ESCALATION_SOS
};

static lv_obj_t *tileview;
static lv_obj_t *home_tile;
static lv_obj_t *home_time_label;
static lv_obj_t *home_status_pill;
static lv_obj_t *home_status_label;
static lv_obj_t *home_bpm_label;
static lv_obj_t *home_bpm_unit;
static lv_obj_t *home_pulse_mark;
static lv_obj_t *home_spo2_label;
static lv_obj_t *home_battery_label;
static lv_obj_t *storage_status_label;
static lv_obj_t *brightness_slider;
static lv_obj_t *brightness_value_label;
static lv_obj_t *auto_night_switch;
static lv_obj_t *night_status_label;
static lv_obj_t *timeout_value_label;
static lv_obj_t *timeout_overlay;

static lv_obj_t *demo_panel;
static lv_obj_t *state_overlay;
static lv_obj_t *state_frame;
static lv_obj_t *state_stage_label;
static lv_obj_t *state_title_label;
static lv_obj_t *state_countdown_label;
static lv_obj_t *state_unit_label;
static lv_obj_t *state_message_label;

static lv_obj_t *splash_overlay;
static lv_obj_t *splash_wordmark;
static void *splash_canvas_buffer;
static bool splash_uses_canvas;
static lv_timer_t *splash_timer;

static lv_timer_t *escalation_timer;
static EscalationStage escalation_stage = ESCALATION_NONE;
static int16_t escalation_seconds = 0;
static uint16_t ui_seconds = 0;
static uint32_t last_feed_ms = 0;
static int16_t bpm = 68;
static int16_t spo2 = 97;
static Preferences display_preferences;
static bool preferences_ready = false;
static bool auto_night_enabled = true;
static uint8_t day_brightness = DEFAULT_DAY_BRIGHTNESS;
static uint8_t night_brightness = DEFAULT_NIGHT_BRIGHTNESS;
static uint16_t screen_timeout_seconds = DEFAULT_TIMEOUT_SECONDS;
static uint32_t last_display_activity_ms = 0;
static bool display_dimmed = false;
static bool display_asleep = false;

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return label;
}

static lv_obj_t *make_panel(lv_obj_t *parent, int16_t x, int16_t y, int16_t width, int16_t height)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, width, height);
    lv_obj_set_style_bg_color(panel, COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 18, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

static void create_stat_card(lv_obj_t *parent, int16_t x, int16_t y, int16_t width,
                             const char *title, const char *value, lv_color_t value_color,
                             lv_obj_t **value_label)
{
    lv_obj_t *card = make_panel(parent, x, y, width, 68);
    lv_obj_t *title_label = make_label(card, title, &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_set_pos(title_label, 14, 8);

    lv_obj_t *metric_label = make_label(card, value, &lv_font_montserrat_24, value_color);
    lv_obj_set_pos(metric_label, 14, 34);
    if(value_label != NULL) {
        *value_label = metric_label;
    }
}

static int16_t battery_percent_from_voltage(float voltage)
{
    struct BatteryPoint {
        float voltage;
        uint8_t percent;
    };
    static const BatteryPoint curve[] = {
        {3.30f, 0}, {3.45f, 5}, {3.60f, 15}, {3.70f, 30},
        {3.80f, 50}, {3.90f, 65}, {4.00f, 80}, {4.10f, 90}, {4.20f, 100}
    };

    if(voltage < 2.5f || voltage > 4.5f) {
        return -1;
    }
    if(voltage <= curve[0].voltage) {
        return curve[0].percent;
    }
    if(voltage >= curve[8].voltage) {
        return curve[8].percent;
    }

    for(uint8_t i = 1; i < 9; i++) {
        if(voltage <= curve[i].voltage) {
            float span = curve[i].voltage - curve[i - 1].voltage;
            float fraction = (voltage - curve[i - 1].voltage) / span;
            return curve[i - 1].percent +
                   static_cast<int16_t>((curve[i].percent - curve[i - 1].percent) * fraction);
        }
    }
    return 100;
}

/* Gelang mengirim satu baris per detik. Lewat 5 detik tanpa kabar berarti
 * UART mati atau gelang reboot — panel demo diambil alih lagi supaya layar
 * tetap bisa diperagakan ke juri tanpa gelang menyala. */
static bool ui_is_live(void)
{
    return last_feed_ms != 0 && millis() - last_feed_ms < 5000;
}

static void update_hardware_status_labels(void)
{
    if(home_battery_label != NULL) {
        int16_t percent = battery_percent_from_voltage(BAT_analogVolts);
        if(percent < 0) {
            lv_label_set_text(home_battery_label, "--%");
            lv_obj_set_style_text_color(home_battery_label, COLOR_MUTED, 0);
        } else {
            lv_label_set_text_fmt(home_battery_label, "%d%%", percent);
            lv_color_t color = percent <= 15 ? COLOR_DANGER :
                               (percent <= 30 ? COLOR_WARNING : COLOR_SUCCESS);
            lv_obj_set_style_text_color(home_battery_label, color, 0);
        }
    }

    if(storage_status_label != NULL) {
        bool ready = Repulse_Storage_Ready();
        lv_label_set_text(storage_status_label, ready ? "SD READY" : "NO CARD");
        lv_obj_set_style_text_color(storage_status_label,
                                    ready ? COLOR_SUCCESS : COLOR_WARNING, 0);
    }
}

static void create_page_dots(lv_obj_t *parent, uint8_t active_page, int16_t y)
{
    for(uint8_t i = 0; i < 4; i++) {
        lv_obj_t *dot = lv_obj_create(parent);
        lv_obj_remove_style_all(dot);
        int16_t width = (i == active_page) ? 16 : 6;
        lv_obj_set_size(dot, width, 6);
        lv_obj_set_pos(dot, 164 + (i * 20), y);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, (i == active_page) ? COLOR_PULSE : COLOR_DIVIDER, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    }
}

static bool rtc_is_valid(void)
{
    return datetime.year >= 2024 && datetime.year <= 2069 &&
           datetime.month >= 1 && datetime.month <= 12 &&
           datetime.day >= 1 && datetime.day <= 31 &&
           datetime.hour <= 23 && datetime.minute <= 59;
}

static bool rtc_is_night(void)
{
    return rtc_is_valid() && (datetime.hour >= 19 || datetime.hour < 6);
}

static bool night_profile_is_active(void)
{
    return auto_night_enabled && rtc_is_night();
}

static uint8_t active_brightness(void)
{
    return night_profile_is_active() ? night_brightness : day_brightness;
}

static uint32_t display_dim_ms(void)
{
    if(screen_timeout_seconds == 15) {
        return 10000;
    }
    if(screen_timeout_seconds == 45) {
        return 30000;
    }
    return 15000;
}

static uint32_t display_off_ms(void)
{
    return static_cast<uint32_t>(screen_timeout_seconds) * 1000U;
}

static uint8_t valid_brightness(uint8_t value, uint8_t fallback)
{
    return (value >= 5 && value <= Backlight_MAX) ? value : fallback;
}

static uint16_t valid_timeout(uint16_t value)
{
    return (value == 15 || value == 25 || value == 45) ? value : DEFAULT_TIMEOUT_SECONDS;
}

static void load_display_settings(void)
{
    preferences_ready = display_preferences.begin("repulse-ui", false);
    if(!preferences_ready) {
        return;
    }

    day_brightness = valid_brightness(
        display_preferences.getUChar("dayBright", DEFAULT_DAY_BRIGHTNESS),
        DEFAULT_DAY_BRIGHTNESS);
    night_brightness = valid_brightness(
        display_preferences.getUChar("nightBright", DEFAULT_NIGHT_BRIGHTNESS),
        DEFAULT_NIGHT_BRIGHTNESS);
    auto_night_enabled = display_preferences.getBool("autoNight", true);
    screen_timeout_seconds = valid_timeout(
        display_preferences.getUShort("timeout", DEFAULT_TIMEOUT_SECONDS));
}

static void apply_display_brightness(void)
{
    uint8_t target = active_brightness();
    if(escalation_stage != ESCALATION_NONE) {
        if(target < ALERT_BRIGHTNESS_MIN) {
            target = ALERT_BRIGHTNESS_MIN;
        }
    } else if(display_asleep) {
        target = 0;
    } else if(display_dimmed && target > IDLE_DIM_BRIGHTNESS) {
        target = IDLE_DIM_BRIGHTNESS;
    }

    if(LCD_Backlight != target) {
        Set_Backlight(target);
    }
}

static void update_display_settings_labels(void)
{
    uint8_t brightness = active_brightness();
    if(brightness_value_label != NULL) {
        lv_label_set_text_fmt(brightness_value_label, "%u%%", brightness);
    }
    if(brightness_slider != NULL && lv_slider_get_value(brightness_slider) != brightness) {
        lv_slider_set_value(brightness_slider, brightness, LV_ANIM_ON);
    }
    if(auto_night_switch != NULL) {
        if(auto_night_enabled) {
            lv_obj_add_state(auto_night_switch, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(auto_night_switch, LV_STATE_CHECKED);
        }
    }
    if(timeout_value_label != NULL) {
        lv_label_set_text_fmt(timeout_value_label, "%u SEC  >", screen_timeout_seconds);
    }
    if(night_status_label == NULL) {
        return;
    }

    if(!auto_night_enabled) {
        lv_label_set_text(night_status_label, "OFF");
        lv_obj_set_style_text_color(night_status_label, COLOR_MUTED, 0);
    } else if(!rtc_is_valid()) {
        lv_label_set_text(night_status_label, "SET RTC");
        lv_obj_set_style_text_color(night_status_label, COLOR_WARNING, 0);
    } else if(rtc_is_night()) {
        lv_label_set_text(night_status_label, "NIGHT ACTIVE");
        lv_obj_set_style_text_color(night_status_label, COLOR_SLEEP, 0);
    } else {
        lv_label_set_text(night_status_label, "19:00 - 06:00");
        lv_obj_set_style_text_color(night_status_label, COLOR_SUCCESS, 0);
    }
}

static void wake_display(void)
{
    last_display_activity_ms = lv_tick_get();
    display_dimmed = false;
    display_asleep = false;
    Lvgl_Set_Wake_Touch_Only(false);
    apply_display_brightness();
}

static void brightness_slider_event(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    if(code == LV_EVENT_PRESSED) {
        lv_obj_clear_flag(tileview, LV_OBJ_FLAG_SCROLLABLE);
    } else if(code == LV_EVENT_VALUE_CHANGED) {
        uint8_t brightness = static_cast<uint8_t>(lv_slider_get_value(lv_event_get_target(event)));
        if(night_profile_is_active()) {
            night_brightness = brightness;
        } else {
            day_brightness = brightness;
        }
        wake_display();
        update_display_settings_labels();
    } else if(code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        lv_obj_add_flag(tileview, LV_OBJ_FLAG_SCROLLABLE);
        if(preferences_ready) {
            if(night_profile_is_active()) {
                display_preferences.putUChar("nightBright", night_brightness);
            } else {
                display_preferences.putUChar("dayBright", day_brightness);
            }
        }
    }
}

static void auto_night_event(lv_event_t *event)
{
    if(lv_event_get_code(event) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    auto_night_enabled = lv_obj_has_state(lv_event_get_target(event), LV_STATE_CHECKED);
    if(preferences_ready) {
        display_preferences.putBool("autoNight", auto_night_enabled);
    }
    wake_display();
    update_display_settings_labels();
}

static void close_timeout_overlay(void)
{
    if(timeout_overlay == NULL) {
        return;
    }
    lv_obj_t *overlay = timeout_overlay;
    timeout_overlay = NULL;
    lv_obj_del_async(overlay);
}

static void timeout_choice_event(lv_event_t *event)
{
    if(lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    screen_timeout_seconds = static_cast<uint16_t>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    if(preferences_ready) {
        display_preferences.putUShort("timeout", screen_timeout_seconds);
    }
    wake_display();
    update_display_settings_labels();
    close_timeout_overlay();
}

static void create_timeout_option(lv_obj_t *parent, int16_t y, uint16_t seconds)
{
    bool selected = screen_timeout_seconds == seconds;
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_remove_style_all(button);
    lv_obj_set_pos(button, 75, y);
    lv_obj_set_size(button, 262, 56);
    lv_obj_set_style_bg_color(button, selected ? COLOR_RAISED : COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(button, selected ? COLOR_PULSE : COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(button, selected ? 2 : 1, 0);
    lv_obj_set_style_radius(button, 18, 0);
    lv_obj_set_style_bg_color(button, COLOR_RAISED, LV_STATE_PRESSED);
    lv_obj_add_event_cb(button, timeout_choice_event, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(static_cast<intptr_t>(seconds)));

    lv_obj_t *label = make_label(button, "", &lv_font_montserrat_18,
                                 selected ? COLOR_PULSE : COLOR_PRIMARY);
    lv_label_set_text_fmt(label, "%u SECONDS", seconds);
    lv_obj_center(label);
}

static void show_timeout_overlay(void)
{
    if(timeout_overlay != NULL) {
        return;
    }
    wake_display();

    timeout_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(timeout_overlay);
    lv_obj_set_size(timeout_overlay, 412, 412);
    lv_obj_set_pos(timeout_overlay, 0, 0);
    lv_obj_set_style_bg_color(timeout_overlay, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(timeout_overlay, LV_OPA_COVER, 0);
    lv_obj_add_flag(timeout_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(timeout_overlay, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = make_label(timeout_overlay, "SCREEN TIMEOUT",
                                 &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_t *subtitle = make_label(timeout_overlay, "CHOOSE DISPLAY DURATION",
                                    &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 85);

    create_timeout_option(timeout_overlay, 125, 15);
    create_timeout_option(timeout_overlay, 191, 25);
    create_timeout_option(timeout_overlay, 257, 45);
}

static void timeout_panel_event(lv_event_t *event)
{
    if(lv_event_get_code(event) == LV_EVENT_CLICKED) {
        show_timeout_overlay();
    }
}

static void display_power_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if(Lvgl_Take_Touch_Activity()) {
        wake_display();
    }

    if(escalation_stage != ESCALATION_NONE) {
        wake_display();
        return;
    }

    uint32_t idle_ms = lv_tick_get() - last_display_activity_ms;
    if(idle_ms >= display_off_ms()) {
        if(!display_asleep) {
            display_asleep = true;
            display_dimmed = false;
            Lvgl_Set_Wake_Touch_Only(true);
            apply_display_brightness();
        }
    } else if(idle_ms >= display_dim_ms() && !display_dimmed) {
        display_dimmed = true;
        apply_display_brightness();
    } else if(!display_asleep) {
        apply_display_brightness();
    }
}

static void close_demo_panel(void)
{
    if(demo_panel != NULL) {
        lv_obj_del(demo_panel);
        demo_panel = NULL;
    }
}

static void reset_state_overlay(void)
{
    if(escalation_timer != NULL) {
        lv_timer_del(escalation_timer);
        escalation_timer = NULL;
    }
    escalation_stage = ESCALATION_NONE;
    escalation_seconds = 0;

    if(state_overlay != NULL) {
        lv_obj_del(state_overlay);
        state_overlay = NULL;
        state_frame = NULL;
        state_stage_label = NULL;
        state_title_label = NULL;
        state_countdown_label = NULL;
        state_unit_label = NULL;
        state_message_label = NULL;
    }
}

static void state_long_press_event(lv_event_t *event)
{
    if(lv_event_get_code(event) == LV_EVENT_LONG_PRESSED) {
        reset_state_overlay();
    }
}

static void create_state_overlay(lv_color_t accent)
{
    reset_state_overlay();

    state_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(state_overlay);
    lv_obj_set_size(state_overlay, 412, 412);
    lv_obj_set_pos(state_overlay, 0, 0);
    lv_obj_set_style_bg_color(state_overlay, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(state_overlay, LV_OPA_COVER, 0);
    lv_obj_add_flag(state_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(state_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(state_overlay, state_long_press_event, LV_EVENT_LONG_PRESSED, NULL);

    state_frame = lv_obj_create(state_overlay);
    lv_obj_remove_style_all(state_frame);
    lv_obj_set_size(state_frame, 362, 362);
    lv_obj_align(state_frame, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(state_frame, COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(state_frame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(state_frame, accent, 0);
    lv_obj_set_style_border_opa(state_frame, LV_OPA_60, 0);
    lv_obj_set_style_border_width(state_frame, 2, 0);
    lv_obj_set_style_radius(state_frame, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(state_frame, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    state_stage_label = make_label(state_overlay, "", &lv_font_montserrat_18, accent);
    lv_obj_align(state_stage_label, LV_ALIGN_TOP_MID, 0, 57);

    state_title_label = make_label(state_overlay, "", &lv_font_montserrat_32, COLOR_PRIMARY);
    lv_obj_set_style_text_letter_space(state_title_label, 1, 0);
    lv_obj_align(state_title_label, LV_ALIGN_TOP_MID, 0, 94);

    state_countdown_label = make_label(state_overlay, "", &lv_font_montserrat_48, accent);
    lv_obj_align(state_countdown_label, LV_ALIGN_TOP_MID, 0, 148);

    state_unit_label = make_label(state_overlay, "SECONDS", &lv_font_montserrat_18, COLOR_SECONDARY);
    lv_obj_align(state_unit_label, LV_ALIGN_TOP_MID, 0, 205);

    state_message_label = make_label(state_overlay, "", &lv_font_montserrat_18, COLOR_SECONDARY);
    lv_obj_set_width(state_message_label, 292);
    lv_obj_set_style_text_align(state_message_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(state_message_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(state_message_label, LV_ALIGN_TOP_MID, 0, 249);

    lv_obj_t *note = make_label(state_overlay, "DEMO MODE: HOLD TO RESET",
                                &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(note, LV_ALIGN_BOTTOM_MID, 0, -53);
}

static void render_escalation_stage(void)
{
    lv_color_t accent = COLOR_BREATH;
    if(escalation_stage == ESCALATION_STAGE_3) {
        accent = COLOR_DANGER;
    } else if(escalation_stage == ESCALATION_SOS) {
        accent = COLOR_SOS;
    }

    lv_obj_set_style_border_color(state_frame, accent, 0);
    lv_obj_set_style_text_color(state_stage_label, accent, 0);
    lv_obj_set_style_text_color(state_countdown_label, accent, 0);

    if(escalation_stage == ESCALATION_STAGE_2) {
        lv_label_set_text(state_stage_label, "STAGE 2");
        lv_label_set_text(state_title_label, "SOFT VIBRATION");
        lv_label_set_text_fmt(state_countdown_label, "%d", escalation_seconds);
        lv_obj_clear_flag(state_countdown_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(state_unit_label, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(state_message_label, "Move naturally to confirm you are okay");
    } else if(escalation_stage == ESCALATION_STAGE_3) {
        lv_label_set_text(state_stage_label, "STAGE 3");
        lv_label_set_text(state_title_label, "ALERT");
        lv_label_set_text_fmt(state_countdown_label, "%d", escalation_seconds);
        lv_obj_clear_flag(state_countdown_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(state_unit_label, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(state_message_label, "Movement can stop escalation");
    } else {
        lv_label_set_text(state_stage_label, "ESCALATION COMPLETE");
        lv_label_set_text(state_title_label, "SOS ACTIVE");
        lv_obj_add_flag(state_countdown_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(state_unit_label, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(state_message_label,
                          "Open phone to review and send\nNothing was sent automatically");
    }
}

static void escalation_timer_cb(lv_timer_t *timer)
{
    if(escalation_stage == ESCALATION_STAGE_2 || escalation_stage == ESCALATION_STAGE_3) {
        escalation_seconds--;
        if(escalation_seconds > 0) {
            lv_label_set_text_fmt(state_countdown_label, "%d", escalation_seconds);
            return;
        }
    }

    if(escalation_stage == ESCALATION_STAGE_2) {
        escalation_stage = ESCALATION_STAGE_3;
        escalation_seconds = 30;
        render_escalation_stage();
    } else if(escalation_stage == ESCALATION_STAGE_3) {
        escalation_stage = ESCALATION_SOS;
        escalation_seconds = 0;
        escalation_timer = NULL;
        render_escalation_stage();
        lv_timer_del(timer);
    }
}

/* `run_countdown` hanya benar untuk panel demo. Saat gelang tersambung,
 * tahap datang dari sana dan hitung mundur lokal justru membuat dua otoritas
 * yang bisa berselisih — persis yang dilarang PRD §4.1 Aturan 1. */
static void show_escalation(EscalationStage initial_stage, bool run_countdown)
{
    wake_display();
    close_demo_panel();
    create_state_overlay(initial_stage == ESCALATION_SOS ? COLOR_SOS : COLOR_BREATH);
    escalation_stage = initial_stage;
    escalation_seconds = (initial_stage == ESCALATION_STAGE_2) ? 15 : 0;
    render_escalation_stage();

    if(run_countdown && initial_stage != ESCALATION_SOS) {
        escalation_timer = lv_timer_create(escalation_timer_cb, 1000, NULL);
    }
}

static void show_offline_state(void)
{
    close_demo_panel();
    create_state_overlay(COLOR_WARNING);
    lv_label_set_text(state_stage_label, "OFFLINE");
    lv_label_set_text(state_title_label, "PHONE OUT OF REACH");
    lv_obj_set_style_text_font(state_title_label, &lv_font_montserrat_24, 0);
    lv_label_set_text(state_countdown_label, "3");
    lv_label_set_text(state_unit_label, "BUFFERED EVENTS");
    lv_label_set_text(state_message_label, "Monitoring continues on your watch");
}

static void apply_comfort_mode(bool enabled)
{
    lv_color_t accent = enabled ? COLOR_SUCCESS : COLOR_PULSE;
    lv_label_set_text(home_status_label, enabled ? "COMFORT" : "MONITORING");
    lv_obj_set_style_border_color(home_status_pill, accent, 0);
    lv_obj_set_style_text_color(home_status_label, accent, 0);
    lv_obj_set_style_text_color(home_bpm_label, accent, 0);
    lv_obj_set_style_text_color(home_bpm_unit, accent, 0);
    lv_obj_set_style_bg_color(home_pulse_mark, accent, 0);
}

static void demo_button_event(lv_event_t *event)
{
    if(lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    DemoAction action = static_cast<DemoAction>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    switch(action) {
        case DEMO_NORMAL:
            apply_comfort_mode(false);
            close_demo_panel();
            break;
        case DEMO_COMFORT:
            apply_comfort_mode(true);
            close_demo_panel();
            break;
        case DEMO_OFFLINE:
            show_offline_state();
            break;
        case DEMO_ALERT:
            show_escalation(ESCALATION_STAGE_2, true);
            break;
        case DEMO_SOS:
            show_escalation(ESCALATION_SOS, true);
            break;
        case DEMO_BACK:
            close_demo_panel();
            break;
    }
}

static void create_demo_button(lv_obj_t *parent, int16_t x, int16_t y, const char *text,
                               DemoAction action, lv_color_t accent)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_remove_style_all(button);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, 146, 54);
    lv_obj_set_style_bg_color(button, COLOR_RAISED, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(button, accent, 0);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_radius(button, 18, 0);
    lv_obj_set_style_bg_color(button, COLOR_SURFACE, LV_STATE_PRESSED);
    lv_obj_add_event_cb(button, demo_button_event, LV_EVENT_CLICKED,
                        reinterpret_cast<void *>(static_cast<intptr_t>(action)));

    lv_obj_t *label = make_label(button, text, &lv_font_montserrat_18, accent);
    lv_obj_center(label);
}

static void show_demo_panel(void)
{
    if(demo_panel != NULL || state_overlay != NULL || splash_overlay != NULL) {
        return;
    }

    demo_panel = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(demo_panel);
    lv_obj_set_size(demo_panel, 412, 412);
    lv_obj_set_pos(demo_panel, 0, 0);
    lv_obj_set_style_bg_color(demo_panel, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(demo_panel, LV_OPA_COVER, 0);
    lv_obj_add_flag(demo_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(demo_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *frame = lv_obj_create(demo_panel);
    lv_obj_remove_style_all(frame);
    lv_obj_set_pos(frame, 38, 34);
    lv_obj_set_size(frame, 336, 344);
    lv_obj_set_style_bg_color(frame, COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(frame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(frame, COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(frame, 1, 0);
    lv_obj_set_style_radius(frame, 46, 0);
    lv_obj_clear_flag(frame, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = make_label(demo_panel, "DEMO MODE", &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 55);
    lv_obj_t *subtitle = make_label(demo_panel, "Choose a prototype state",
                                    &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 86);

    create_demo_button(demo_panel, 54, 124, "NORMAL", DEMO_NORMAL, COLOR_PULSE);
    create_demo_button(demo_panel, 212, 124, "COMFORT", DEMO_COMFORT, COLOR_SUCCESS);
    create_demo_button(demo_panel, 54, 190, "OFFLINE", DEMO_OFFLINE, COLOR_WARNING);
    create_demo_button(demo_panel, 212, 190, "ALERT", DEMO_ALERT, COLOR_DANGER);
    create_demo_button(demo_panel, 54, 256, "SOS", DEMO_SOS, COLOR_SOS);
    create_demo_button(demo_panel, 212, 256, "BACK", DEMO_BACK, COLOR_SECONDARY);
}

static void page_long_press_event(lv_event_t *event)
{
    if(lv_event_get_code(event) == LV_EVENT_LONG_PRESSED) {
        show_demo_panel();
    }
}

static void create_home_page(lv_obj_t *parent)
{
    home_time_label = make_label(parent, "--:--", &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(home_time_label, LV_ALIGN_TOP_MID, 0, 39);

    home_status_pill = lv_obj_create(parent);
    lv_obj_remove_style_all(home_status_pill);
    lv_obj_set_size(home_status_pill, 154, 36);
    lv_obj_align(home_status_pill, LV_ALIGN_TOP_MID, 0, 79);
    lv_obj_set_style_bg_color(home_status_pill, COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(home_status_pill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(home_status_pill, COLOR_PULSE, 0);
    lv_obj_set_style_border_width(home_status_pill, 1, 0);
    lv_obj_set_style_radius(home_status_pill, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(home_status_pill, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    home_pulse_mark = lv_obj_create(home_status_pill);
    lv_obj_remove_style_all(home_pulse_mark);
    lv_obj_set_size(home_pulse_mark, 8, 8);
    lv_obj_align(home_pulse_mark, LV_ALIGN_LEFT_MID, 16, 0);
    lv_obj_set_style_bg_color(home_pulse_mark, COLOR_PULSE, 0);
    lv_obj_set_style_bg_opa(home_pulse_mark, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(home_pulse_mark, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(home_pulse_mark, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    home_status_label = make_label(home_status_pill, "MONITORING", &lv_font_montserrat_18, COLOR_PULSE);
    lv_obj_align(home_status_label, LV_ALIGN_CENTER, 7, 0);

    lv_obj_t *metric_title = make_label(parent, "RESTING PULSE", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(metric_title, LV_ALIGN_TOP_MID, 0, 127);

    home_bpm_label = make_label(parent, "68", &lv_font_montserrat_48, COLOR_PULSE);
    lv_obj_align(home_bpm_label, LV_ALIGN_TOP_MID, -15, 151);
    home_bpm_unit = make_label(parent, "BPM", &lv_font_montserrat_18, COLOR_PULSE);
    lv_obj_align_to(home_bpm_unit, home_bpm_label, LV_ALIGN_OUT_RIGHT_BOTTOM, 8, -8);

    create_stat_card(parent, 53, 235, 147, "SPO2", "97%", COLOR_PRIMARY, &home_spo2_label);
    create_stat_card(parent, 212, 235, 147, "BATTERY", "--%", COLOR_MUTED,
                     &home_battery_label);

    create_page_dots(parent, 0, 333);
    lv_obj_t *notice = make_label(parent, "Not a medical device", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(notice, LV_ALIGN_BOTTOM_MID, 0, -28);
}

static void create_body_page(lv_obj_t *parent)
{
    lv_obj_t *title = make_label(parent, "BODY SIGNALS", &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 42);
    lv_obj_t *subtitle = make_label(parent, "Live rhythm snapshot", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 73);

    lv_obj_t *rr_title = make_label(parent, "R-R INTERVAL", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(rr_title, LV_ALIGN_TOP_MID, 0, 111);
    lv_obj_t *rr_value = make_label(parent, "882", &lv_font_montserrat_48, COLOR_BREATH);
    lv_obj_align(rr_value, LV_ALIGN_TOP_MID, -13, 137);
    lv_obj_t *rr_unit = make_label(parent, "ms", &lv_font_montserrat_18, COLOR_BREATH);
    lv_obj_align_to(rr_unit, rr_value, LV_ALIGN_OUT_RIGHT_BOTTOM, 7, -8);

    create_stat_card(parent, 53, 213, 147, "MOTION", "LOW", COLOR_SUCCESS, NULL);
    create_stat_card(parent, 212, 213, 147, "POSITION", "LEFT", COLOR_SLEEP, NULL);
    create_stat_card(parent, 53, 289, 147, "SIGNAL", "GOOD", COLOR_SUCCESS, NULL);
    create_stat_card(parent, 212, 289, 147, "BASELINE", "62", COLOR_PRIMARY, NULL);
    create_page_dots(parent, 1, 366);
}

static void create_sleep_page(lv_obj_t *parent)
{
    lv_obj_t *title = make_label(parent, "SLEEP", &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 43);
    lv_obj_t *subtitle = make_label(parent, "CURRENT SESSION", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 79);

    lv_obj_t *duration = make_label(parent, "02:14", &lv_font_montserrat_48, COLOR_SLEEP);
    lv_obj_align(duration, LV_ALIGN_TOP_MID, 0, 105);

    lv_obj_t *sleep_pill = lv_obj_create(parent);
    lv_obj_remove_style_all(sleep_pill);
    lv_obj_set_size(sleep_pill, 150, 36);
    lv_obj_align(sleep_pill, LV_ALIGN_TOP_MID, 0, 163);
    lv_obj_set_style_bg_color(sleep_pill, COLOR_SURFACE, 0);
    lv_obj_set_style_bg_opa(sleep_pill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(sleep_pill, COLOR_SLEEP, 0);
    lv_obj_set_style_border_width(sleep_pill, 1, 0);
    lv_obj_set_style_radius(sleep_pill, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(sleep_pill, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *stage = make_label(sleep_pill, "DEEP SLEEP", &lv_font_montserrat_18, COLOR_SLEEP);
    lv_obj_center(stage);

    create_stat_card(parent, 53, 217, 147, "MOVEMENT", "LOW", COLOR_SUCCESS, NULL);
    create_stat_card(parent, 212, 217, 147, "POSITION", "LEFT", COLOR_SLEEP, NULL);

    lv_obj_t *storage = make_panel(parent, 53, 293, 306, 58);
    lv_obj_t *storage_title = make_label(storage, "STORAGE", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(storage_title, LV_ALIGN_LEFT_MID, 15, 0);
    storage_status_label = make_label(storage, "NO CARD", &lv_font_montserrat_18, COLOR_WARNING);
    lv_obj_align(storage_status_label, LV_ALIGN_RIGHT_MID, -15, 0);
    create_page_dots(parent, 2, 366);
}

static void create_settings_page(lv_obj_t *parent)
{
    lv_obj_t *title = make_label(parent, "SETTINGS", &lv_font_montserrat_24, COLOR_PRIMARY);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 42);
    lv_obj_t *subtitle = make_label(parent, "DISPLAY", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 76);

    lv_obj_t *brightness_title = make_label(parent, "BRIGHTNESS", &lv_font_montserrat_18, COLOR_SECONDARY);
    lv_obj_set_pos(brightness_title, 67, 116);
    brightness_value_label = make_label(parent, "22%", &lv_font_montserrat_18, COLOR_PULSE);
    lv_obj_align(brightness_value_label, LV_ALIGN_TOP_RIGHT, -67, 116);

    brightness_slider = lv_slider_create(parent);
    lv_obj_set_pos(brightness_slider, 68, 151);
    lv_obj_set_size(brightness_slider, 276, 28);
    lv_slider_set_range(brightness_slider, 5, Backlight_MAX);
    lv_slider_set_value(brightness_slider, active_brightness(), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(brightness_slider, COLOR_DIVIDER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(brightness_slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(brightness_slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(brightness_slider, COLOR_PULSE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(brightness_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(brightness_slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(brightness_slider, COLOR_PRIMARY, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(brightness_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_color(brightness_slider, COLOR_BASE, LV_PART_KNOB);
    lv_obj_set_style_border_width(brightness_slider, 3, LV_PART_KNOB);
    lv_obj_set_style_pad_all(brightness_slider, 5, LV_PART_KNOB);
    lv_obj_add_event_cb(brightness_slider, brightness_slider_event, LV_EVENT_ALL, NULL);

    lv_obj_t *night_panel = make_panel(parent, 53, 209, 306, 62);
    lv_obj_t *night_title = make_label(night_panel, "AUTO NIGHT", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_set_pos(night_title, 15, 7);
    night_status_label = make_label(night_panel, "19:00 - 06:00", &lv_font_montserrat_18, COLOR_SUCCESS);
    lv_obj_set_pos(night_status_label, 15, 32);

    auto_night_switch = lv_switch_create(night_panel);
    lv_obj_set_size(auto_night_switch, 52, 30);
    lv_obj_align(auto_night_switch, LV_ALIGN_RIGHT_MID, -15, 0);
    lv_obj_set_style_bg_color(auto_night_switch, COLOR_DIVIDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(auto_night_switch, COLOR_PULSE,
                              LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(auto_night_switch, COLOR_PRIMARY, LV_PART_KNOB);
    if(auto_night_enabled) {
        lv_obj_add_state(auto_night_switch, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(auto_night_switch, auto_night_event, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *timeout_panel = make_panel(parent, 53, 283, 306, 62);
    lv_obj_add_flag(timeout_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(timeout_panel, COLOR_RAISED, LV_STATE_PRESSED);
    lv_obj_add_event_cb(timeout_panel, timeout_panel_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *timeout_title = make_label(timeout_panel, "SCREEN OFF", &lv_font_montserrat_18, COLOR_MUTED);
    lv_obj_align(timeout_title, LV_ALIGN_LEFT_MID, 15, 0);
    timeout_value_label = make_label(timeout_panel, "25 SEC  >", &lv_font_montserrat_18, COLOR_PRIMARY);
    lv_obj_align(timeout_value_label, LV_ALIGN_RIGHT_MID, -15, 0);

    create_page_dots(parent, 3, 366);
    update_display_settings_labels();
}

static void update_clock_label(void)
{
    if(!rtc_is_valid()) {
        lv_label_set_text(home_time_label, "--:--");
        return;
    }

    char time_text[6];
    snprintf(time_text, sizeof(time_text), "%02u:%02u", datetime.hour, datetime.minute);
    lv_label_set_text(home_time_label, time_text);
}

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    static const int8_t bpm_delta[] = {1, 0, -1, 1, -2, 1, 0, 1, -1, 0};
    static uint8_t bpm_step = 0;
    static const int8_t spo2_values[] = {97, 97, 98, 97};
    static uint8_t spo2_step = 0;

    if(ui_is_live()) {
        update_clock_label();
        update_display_settings_labels();
        update_hardware_status_labels();
        return;
    }

    bpm += bpm_delta[bpm_step];
    bpm_step = (bpm_step + 1) % (sizeof(bpm_delta) / sizeof(bpm_delta[0]));
    if(bpm < 66) bpm = 66;
    if(bpm > 71) bpm = 71;
    lv_label_set_text_fmt(home_bpm_label, "%d", bpm);
    lv_obj_align(home_bpm_label, LV_ALIGN_TOP_MID, -15, 151);
    lv_obj_align_to(home_bpm_unit, home_bpm_label, LV_ALIGN_OUT_RIGHT_BOTTOM, 8, -8);

    ui_seconds++;
    if((ui_seconds % 15) == 0) {
        spo2 = spo2_values[spo2_step];
        spo2_step = (spo2_step + 1) % (sizeof(spo2_values) / sizeof(spo2_values[0]));
        lv_label_set_text_fmt(home_spo2_label, "%d%%", spo2);
    }
    update_clock_label();
    update_display_settings_labels();
    update_hardware_status_labels();
}

static lv_point_t wordmark_point(int16_t x, int16_t y)
{
    lv_point_t point;
    point.x = ((int32_t)(x + 2) * SPLASH_CANVAS_WIDTH) / WORDMARK_SOURCE_WIDTH;
    point.y = ((int32_t)(y + 2) * SPLASH_CANVAS_HEIGHT) / WORDMARK_SOURCE_HEIGHT;
    return point;
}

static void draw_partial_canvas_stroke(const lv_point_t *points, uint8_t point_count,
                                       uint16_t progress)
{
    if(progress == 0 || point_count < 2) {
        return;
    }

    float total_length = 0.0f;
    for(uint8_t i = 1; i < point_count; i++) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        total_length += sqrtf((dx * dx) + (dy * dy));
    }

    float remaining = total_length * progress / 1000.0f;
    lv_point_t visible[16];
    uint8_t visible_count = 1;
    visible[0] = points[0];

    for(uint8_t i = 1; i < point_count && visible_count < 16; i++) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        float segment_length = sqrtf((dx * dx) + (dy * dy));
        if(remaining >= segment_length) {
            visible[visible_count++] = points[i];
            remaining -= segment_length;
            continue;
        }

        if(segment_length > 0.0f && remaining > 0.0f) {
            float fraction = remaining / segment_length;
            visible[visible_count].x = points[i - 1].x + (int16_t)(dx * fraction);
            visible[visible_count].y = points[i - 1].y + (int16_t)(dy * fraction);
            visible_count++;
        }
        break;
    }

    if(visible_count < 2) {
        return;
    }

    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = COLOR_IVORY;
    line.width = 2;
    line.round_start = false;
    line.round_end = false;
    lv_canvas_draw_line(splash_wordmark, visible, visible_count, &line);
}

static void draw_wordmark_progress(uint16_t progress)
{
    lv_canvas_fill_bg(splash_wordmark, COLOR_BASE, LV_OPA_COVER);

    lv_point_t r_stem[] = {wordmark_point(2, 0), wordmark_point(2, 100)};
    lv_point_t r_bowl[] = {wordmark_point(2, 2), wordmark_point(36, 2),
                           wordmark_point(46, 4), wordmark_point(54, 9),
                           wordmark_point(59, 17), wordmark_point(61, 27),
                           wordmark_point(59, 37), wordmark_point(54, 45),
                           wordmark_point(46, 50), wordmark_point(36, 52),
                           wordmark_point(2, 52)};
    lv_point_t r_leg[] = {wordmark_point(26, 56), wordmark_point(58, 100)};

    lv_point_t e1_stem[] = {wordmark_point(92, 0), wordmark_point(92, 100)};
    lv_point_t e1_top[] = {wordmark_point(92, 2), wordmark_point(142, 2)};
    lv_point_t e1_mid[] = {wordmark_point(106, 50), wordmark_point(136, 50)};
    lv_point_t e1_bottom[] = {wordmark_point(92, 98), wordmark_point(142, 98)};

    lv_point_t p_stem[] = {wordmark_point(176, 0), wordmark_point(176, 100)};
    lv_point_t p_bowl[] = {wordmark_point(188, 2), wordmark_point(210, 2),
                           wordmark_point(220, 4), wordmark_point(228, 9),
                           wordmark_point(233, 17), wordmark_point(235, 27),
                           wordmark_point(233, 37), wordmark_point(228, 45),
                           wordmark_point(220, 50), wordmark_point(210, 52),
                           wordmark_point(188, 52)};

    lv_point_t u_left[] = {wordmark_point(266, 0), wordmark_point(266, 60)};
    lv_point_t u_curve[] = {wordmark_point(266, 70), wordmark_point(267, 82),
                            wordmark_point(274, 91), wordmark_point(285, 98),
                            wordmark_point(295, 100), wordmark_point(306, 98),
                            wordmark_point(317, 91), wordmark_point(324, 82),
                            wordmark_point(326, 70)};
    lv_point_t u_right[] = {wordmark_point(326, 0), wordmark_point(326, 70)};

    lv_point_t l_stroke[] = {wordmark_point(358, 0), wordmark_point(358, 98),
                             wordmark_point(406, 98)};

    lv_point_t s_upper[] = {wordmark_point(490, 20), wordmark_point(489, 13),
                            wordmark_point(483, 7), wordmark_point(476, 4),
                            wordmark_point(464, 2), wordmark_point(455, 2),
                            wordmark_point(448, 7), wordmark_point(442, 14),
                            wordmark_point(438, 26), wordmark_point(438, 35),
                            wordmark_point(446, 43), wordmark_point(460, 46)};
    lv_point_t s_lower[] = {wordmark_point(468, 54), wordmark_point(476, 54),
                            wordmark_point(484, 58), wordmark_point(488, 65),
                            wordmark_point(490, 74), wordmark_point(490, 83),
                            wordmark_point(483, 91), wordmark_point(475, 96),
                            wordmark_point(464, 98), wordmark_point(455, 98),
                            wordmark_point(446, 95), wordmark_point(441, 89),
                            wordmark_point(438, 80)};

    lv_point_t e2_stem[] = {wordmark_point(522, 0), wordmark_point(522, 100)};
    lv_point_t e2_top[] = {wordmark_point(522, 2), wordmark_point(572, 2)};
    lv_point_t e2_mid[] = {wordmark_point(536, 50), wordmark_point(566, 50)};
    lv_point_t e2_bottom[] = {wordmark_point(522, 98), wordmark_point(572, 98)};

    draw_partial_canvas_stroke(r_stem, 2, progress);
    draw_partial_canvas_stroke(r_bowl, 11, progress);
    draw_partial_canvas_stroke(r_leg, 2, progress);
    draw_partial_canvas_stroke(e1_stem, 2, progress);
    draw_partial_canvas_stroke(e1_top, 2, progress);
    draw_partial_canvas_stroke(e1_mid, 2, progress);
    draw_partial_canvas_stroke(e1_bottom, 2, progress);
    draw_partial_canvas_stroke(p_stem, 2, progress);
    draw_partial_canvas_stroke(p_bowl, 11, progress);
    draw_partial_canvas_stroke(u_left, 2, progress);
    draw_partial_canvas_stroke(u_curve, 9, progress);
    draw_partial_canvas_stroke(u_right, 2, progress);
    draw_partial_canvas_stroke(l_stroke, 3, progress);
    draw_partial_canvas_stroke(s_upper, 12, progress);
    draw_partial_canvas_stroke(s_lower, 13, progress);
    draw_partial_canvas_stroke(e2_stem, 2, progress);
    draw_partial_canvas_stroke(e2_top, 2, progress);
    draw_partial_canvas_stroke(e2_mid, 2, progress);
    draw_partial_canvas_stroke(e2_bottom, 2, progress);
}

static void splash_draw_progress_cb(void *object, int32_t value)
{
    (void)object;
    if(splash_uses_canvas && splash_wordmark != NULL) {
        draw_wordmark_progress(static_cast<uint16_t>(value));
    }
}

static void splash_child_fade_cb(void *object, int32_t value)
{
    lv_obj_set_style_opa(static_cast<lv_obj_t *>(object), static_cast<lv_opa_t>(value), 0);
}

static void splash_background_fade_cb(void *object, int32_t value)
{
    lv_obj_set_style_bg_opa(static_cast<lv_obj_t *>(object), static_cast<lv_opa_t>(value), 0);
}

static void splash_fade_ready_cb(lv_anim_t *animation)
{
    (void)animation;
    if(splash_overlay != NULL) {
        lv_obj_del(splash_overlay);
        splash_overlay = NULL;
        splash_wordmark = NULL;
    }
    if(splash_canvas_buffer != NULL) {
        heap_caps_free(splash_canvas_buffer);
        splash_canvas_buffer = NULL;
    }
}

static void start_splash_fade(void)
{
    lv_anim_t wordmark_anim;
    lv_anim_init(&wordmark_anim);
    lv_anim_set_var(&wordmark_anim, splash_wordmark);
    lv_anim_set_exec_cb(&wordmark_anim, splash_child_fade_cb);
    lv_anim_set_values(&wordmark_anim, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_time(&wordmark_anim, 500);
    lv_anim_set_path_cb(&wordmark_anim, lv_anim_path_ease_in_out);
    lv_anim_start(&wordmark_anim);

    lv_anim_t background_anim;
    lv_anim_init(&background_anim);
    lv_anim_set_var(&background_anim, splash_overlay);
    lv_anim_set_exec_cb(&background_anim, splash_background_fade_cb);
    lv_anim_set_values(&background_anim, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_time(&background_anim, 500);
    lv_anim_set_path_cb(&background_anim, lv_anim_path_ease_in_out);
    lv_anim_set_ready_cb(&background_anim, splash_fade_ready_cb);
    lv_anim_start(&background_anim);
}

static void splash_dwell_timer_cb(lv_timer_t *timer)
{
    splash_timer = NULL;
    start_splash_fade();
    lv_timer_del(timer);
}

static void splash_draw_ready_cb(lv_anim_t *animation)
{
    (void)animation;
    if(splash_uses_canvas && splash_wordmark != NULL) {
        draw_wordmark_progress(1000);
    }
    splash_timer = lv_timer_create(splash_dwell_timer_cb,
                                   SPLASH_DWELL_MS - WORDMARK_DRAW_MS, NULL);
}

static void create_splash(void)
{
    splash_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(splash_overlay);
    lv_obj_set_size(splash_overlay, 412, 412);
    lv_obj_set_pos(splash_overlay, 0, 0);
    lv_obj_set_style_bg_color(splash_overlay, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(splash_overlay, LV_OPA_COVER, 0);
    lv_obj_add_flag(splash_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(splash_overlay, LV_OBJ_FLAG_SCROLLABLE);

    size_t canvas_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR(SPLASH_CANVAS_WIDTH, SPLASH_CANVAS_HEIGHT);
    if(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
        splash_canvas_buffer = heap_caps_malloc(canvas_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if(splash_canvas_buffer == NULL) {
        splash_canvas_buffer = heap_caps_malloc(canvas_size, MALLOC_CAP_8BIT);
    }

    splash_uses_canvas = (splash_canvas_buffer != NULL);
    if(splash_uses_canvas) {
        splash_wordmark = lv_canvas_create(splash_overlay);
        lv_canvas_set_buffer(splash_wordmark, splash_canvas_buffer, SPLASH_CANVAS_WIDTH,
                             SPLASH_CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
        lv_canvas_fill_bg(splash_wordmark, COLOR_BASE, LV_OPA_COVER);
        lv_obj_align(splash_wordmark, LV_ALIGN_CENTER, 0, 0);

        lv_anim_t draw_anim;
        lv_anim_init(&draw_anim);
        lv_anim_set_var(&draw_anim, splash_wordmark);
        lv_anim_set_exec_cb(&draw_anim, splash_draw_progress_cb);
        lv_anim_set_values(&draw_anim, 0, 1000);
        lv_anim_set_time(&draw_anim, WORDMARK_DRAW_MS);
        lv_anim_set_path_cb(&draw_anim, lv_anim_path_ease_out);
        lv_anim_set_ready_cb(&draw_anim, splash_draw_ready_cb);
        lv_anim_start(&draw_anim);
    } else {
        splash_wordmark = make_label(splash_overlay, "REPULSE", &lv_font_montserrat_32, COLOR_IVORY);
        lv_obj_set_style_text_letter_space(splash_wordmark, 4, 0);
        lv_obj_align(splash_wordmark, LV_ALIGN_CENTER, 0, 0);
        splash_timer = lv_timer_create(splash_dwell_timer_cb, SPLASH_DWELL_MS, NULL);
    }
}

void Repulse_UI_Feed(uint8_t bpm_value, uint8_t spo2_value, bool worn,
                     uint8_t stage, bool linked)
{
    last_feed_ms = millis();

    if(home_bpm_label != NULL) {
        bpm = worn ? bpm_value : 0;
        if(worn) {
            lv_label_set_text_fmt(home_bpm_label, "%d", bpm);
        } else {
            /* Gelang di atas meja tidak punya denyut, dan itu bukan nol BPM.
             * Menampilkannya sebagai angka adalah kebohongan yang paling
             * mudah dipercaya. */
            lv_label_set_text(home_bpm_label, "--");
        }
        lv_obj_align(home_bpm_label, LV_ALIGN_TOP_MID, -15, 151);
        lv_obj_align_to(home_bpm_unit, home_bpm_label, LV_ALIGN_OUT_RIGHT_BOTTOM, 8, -8);
    }

    if(home_spo2_label != NULL) {
        spo2 = spo2_value;
        if(spo2_value > 0) {
            lv_label_set_text_fmt(home_spo2_label, "%d%%", spo2_value);
        } else {
            lv_label_set_text(home_spo2_label, "--%");   // 0 = tidak valid
        }
    }

    if(home_status_label != NULL && home_status_pill != NULL) {
        const char *text = "MONITORING";
        lv_color_t  accent = COLOR_PULSE;
        if(!linked) {
            text = "OFFLINE";
            accent = COLOR_WARNING;
        } else if(!worn) {
            text = "NOT WORN";
            accent = COLOR_MUTED;
        }
        lv_label_set_text(home_status_label, text);
        lv_obj_set_style_border_color(home_status_pill, accent, 0);
        lv_obj_set_style_text_color(home_status_label, accent, 0);
    }

    EscalationStage wanted = stage >= 4 ? ESCALATION_SOS :
                             stage == 3 ? ESCALATION_STAGE_3 :
                             stage == 2 ? ESCALATION_STAGE_2 : ESCALATION_NONE;
    if(wanted == escalation_stage) {
        return;
    }
    if(wanted == ESCALATION_NONE) {
        reset_state_overlay();       // turun ke 0 = tubuh merespons
    } else {
        show_escalation(wanted, false);
    }
}

void Repulse_UI_Init(void)
{
    load_display_settings();

    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(screen, &lv_font_montserrat_18, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    tileview = lv_tileview_create(screen);
    lv_obj_set_size(tileview, 412, 412);
    lv_obj_set_pos(tileview, 0, 0);
    lv_obj_set_style_bg_color(tileview, COLOR_BASE, 0);
    lv_obj_set_style_bg_opa(tileview, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tileview, 0, 0);
    lv_obj_set_style_pad_all(tileview, 0, 0);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(tileview, page_long_press_event, LV_EVENT_LONG_PRESSED, NULL);

    home_tile = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_RIGHT);
    lv_obj_t *body_tile = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_t *sleep_tile = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    lv_obj_t *settings_tile = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_LEFT);
    lv_obj_t *tiles[] = {home_tile, body_tile, sleep_tile, settings_tile};
    for(uint8_t i = 0; i < 4; i++) {
        lv_obj_set_style_bg_color(tiles[i], COLOR_BASE, 0);
        lv_obj_set_style_bg_opa(tiles[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(tiles[i], 0, 0);
        lv_obj_set_style_pad_all(tiles[i], 0, 0);
        lv_obj_set_scrollbar_mode(tiles[i], LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(tiles[i], page_long_press_event, LV_EVENT_LONG_PRESSED, NULL);
    }

    create_home_page(home_tile);
    create_body_page(body_tile);
    create_sleep_page(sleep_tile);
    create_settings_page(settings_tile);
    apply_comfort_mode(false);
    update_hardware_status_labels();

    last_display_activity_ms = lv_tick_get();
    Lvgl_Set_Wake_Touch_Only(false);
    apply_display_brightness();
    lv_timer_create(ui_timer_cb, 1000, NULL);
    lv_timer_create(display_power_timer_cb, 100, NULL);
    create_splash();
}
