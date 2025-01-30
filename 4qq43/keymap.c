#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_0_255_255,
  HSV_74_255_255,
  HSV_169_255_255,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  MAC_SPOTLIGHT,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
  DANCE_7,
  DANCE_8,
  DANCE_9,
  DANCE_10,
  DANCE_11,
  DANCE_12,
  DANCE_13,
  DANCE_14,
  DANCE_15,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_CAPS,        KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    CW_TOGG,        KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,                                           KC_J,           KC_L,           KC_U,           KC_Y,           KC_SCLN,        KC_BSLS,        
    KC_ESCAPE,      MT(MOD_LALT, KC_A),MT(MOD_LGUI, KC_R),MT(MOD_LCTL, KC_S),MT(MOD_LSFT, KC_T),KC_G,                                           KC_M,           MT(MOD_RSFT, KC_N),MT(MOD_RCTL, KC_E),MT(MOD_RGUI, KC_I),MT(MOD_LALT, KC_O),MAC_SPOTLIGHT,  
    KC_LEFT_SHIFT,  KC_Z,           KC_X,           KC_C,           KC_D,           KC_V,                                           KC_K,           KC_H,           KC_COMMA,       KC_DOT,         KC_SLASH,       KC_RIGHT_SHIFT, 
                                                    LT(2,KC_SPACE), KC_BSPC,                                        KC_TAB,         LT(1,KC_ENTER)
  ),
  [1] = LAYOUT_voyager(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_PLUS,        KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RPRN,        KC_UNDS,        
    KC_EQUAL,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_TRANSPARENT, KC_GRAVE,       KC_TILD,        KC_QUOTE,       KC_DQUO,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_LCBR,        KC_RCBR,        KC_LBRC,        KC_RBRC,        KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 TD(DANCE_10),   TD(DANCE_11),   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, TD(DANCE_0),    TD(DANCE_1),    TD(DANCE_2),    TD(DANCE_3),    TD(DANCE_4),                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, TD(DANCE_5),    TD(DANCE_6),    TD(DANCE_7),    TD(DANCE_8),    TD(DANCE_9),                                    TD(DANCE_12),   TD(DANCE_13),   KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_AUDIO_MUTE,  KC_TRANSPARENT,                                 KC_DOWN,        KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE,                                KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,HSV_0_255_255,  HSV_74_255_255, HSV_169_255_255,                                KC_UP,          KC_LEFT,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_A,           KC_R,           KC_S,           KC_T,           KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_N,           KC_E,           KC_I,           KC_O,           KC_TRANSPARENT, 
    KC_TRANSPARENT, MT(MOD_LCTL, KC_Z),KC_TRANSPARENT, KC_TRANSPARENT, KC_D,           KC_V,                                           KC_K,           KC_H,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    TD(DANCE_14),   TD(DANCE_15),                                   KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const uint16_t PROGMEM combo0[] = { LT(1,KC_ENTER), LT(2,KC_SPACE), COMBO_END};
const uint16_t PROGMEM combo1[] = { KC_5, KC_6, KC_4, KC_7, COMBO_END};
const uint16_t PROGMEM combo2[] = { KC_D, KC_V, KC_K, KC_H, COMBO_END};
const uint16_t PROGMEM combo3[] = { KC_F, KC_U, MT(MOD_RSFT, KC_N), COMBO_END};
const uint16_t PROGMEM combo4[] = { KC_C, MT(MOD_RSFT, KC_N), COMBO_END};
const uint16_t PROGMEM combo5[] = { MT(MOD_RGUI, KC_I), KC_M, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, TT(3)),
    COMBO(combo1, TG(4)),
    COMBO(combo2, TO(0)),
    COMBO(combo3, ST_MACRO_0),
    COMBO(combo4, ST_MACRO_1),
    COMBO(combo5, ST_MACRO_2),
};


extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {94,218,204}, {13,255,255}, {13,255,255}, {13,255,255}, {13,255,255}, {166,193,229}, {94,218,204}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {39,231,229}, {39,231,229}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {94,218,204}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {94,218,204}, {166,193,229}, {13,255,255}, {13,255,255}, {13,255,255}, {13,255,255}, {94,218,204}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {166,193,229}, {94,218,204}, {39,231,229}, {39,231,229} },

    [1] = { {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {167,255,255}, {167,255,255}, {13,255,255}, {13,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {188,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {169,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {152,255,255}, {0,0,0}, {13,255,255}, {13,255,255}, {167,255,255}, {167,255,255}, {0,0,0}, {0,0,0}, {0,0,0} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,0,0}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,245,245}, {0,245,245}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,0,0}, {0,0,0}, {0,0,0}, {166,193,229}, {166,193,229}, {166,193,229}, {0,0,0}, {0,0,0}, {13,255,255}, {13,255,255}, {13,255,255}, {13,255,255}, {74,255,255}, {0,0,0}, {74,255,255}, {74,255,255}, {0,233,107}, {0,233,107}, {0,233,107}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {94,218,204}, {94,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {13,255,255}, {13,255,255}, {13,255,255}, {13,255,255}, {0,0,0}, {94,218,204}, {94,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107}, {0,233,107} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F) SS_TAP(X_U) SS_TAP(X_N) SS_TAP(X_C) SS_TAP(X_T) SS_TAP(X_I) SS_TAP(X_O) SS_TAP(X_N));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_C) SS_TAP(X_O) SS_TAP(X_N) SS_TAP(X_S) SS_TAP(X_T));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_I) SS_TAP(X_M) SS_TAP(X_P) SS_TAP(X_O) SS_TAP(X_R) SS_TAP(X_T));
    }
    break;
    case MAC_SPOTLIGHT:
      HCS(0x221);

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_0_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,255,255);
      }
      return false;
    case HSV_74_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(74,255,255);
      }
      return false;
    case HSV_169_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(169,255,255);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[16];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_1)));
        tap_code16(LALT(LGUI(KC_1)));
        tap_code16(LALT(LGUI(KC_1)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_1)));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_1))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_1)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_1))); register_code16(LALT(LGUI(KC_1)));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_1))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_1)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_1))); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_2)));
        tap_code16(LALT(LGUI(KC_2)));
        tap_code16(LALT(LGUI(KC_2)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_2)));
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_2))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_2)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_2))); register_code16(LALT(LGUI(KC_2)));
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_2))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_2)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_2))); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_3)));
        tap_code16(LALT(LGUI(KC_3)));
        tap_code16(LALT(LGUI(KC_3)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_3)));
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_3))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_3)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_3))); register_code16(LALT(LGUI(KC_3)));
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_3))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_3)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_3))); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_4)));
        tap_code16(LALT(LGUI(KC_4)));
        tap_code16(LALT(LGUI(KC_4)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_4)));
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_4))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_4)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_4))); register_code16(LALT(LGUI(KC_4)));
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_4))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_4)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_4))); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(tap_dance_state_t *state, void *user_data);
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void on_dance_4(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_5)));
        tap_code16(LALT(LGUI(KC_5)));
        tap_code16(LALT(LGUI(KC_5)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_5)));
    }
}

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_5))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_5)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_5))); register_code16(LALT(LGUI(KC_5)));
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_5))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_5)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_5))); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(tap_dance_state_t *state, void *user_data);
void dance_5_finished(tap_dance_state_t *state, void *user_data);
void dance_5_reset(tap_dance_state_t *state, void *user_data);

void on_dance_5(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_6)));
        tap_code16(LALT(LGUI(KC_6)));
        tap_code16(LALT(LGUI(KC_6)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_6)));
    }
}

void dance_5_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_6))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_6)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_6))); register_code16(LALT(LGUI(KC_6)));
    }
}

void dance_5_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_6))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_6)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_6))); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(tap_dance_state_t *state, void *user_data);
void dance_6_finished(tap_dance_state_t *state, void *user_data);
void dance_6_reset(tap_dance_state_t *state, void *user_data);

void on_dance_6(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_7)));
        tap_code16(LALT(LGUI(KC_7)));
        tap_code16(LALT(LGUI(KC_7)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_7)));
    }
}

void dance_6_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_7))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_7)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_7))); register_code16(LALT(LGUI(KC_7)));
    }
}

void dance_6_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_7))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_7)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_7))); break;
    }
    dance_state[6].step = 0;
}
void on_dance_7(tap_dance_state_t *state, void *user_data);
void dance_7_finished(tap_dance_state_t *state, void *user_data);
void dance_7_reset(tap_dance_state_t *state, void *user_data);

void on_dance_7(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_8)));
        tap_code16(LALT(LGUI(KC_8)));
        tap_code16(LALT(LGUI(KC_8)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_8)));
    }
}

void dance_7_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[7].step = dance_step(state);
    switch (dance_state[7].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_8))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_8)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_8))); register_code16(LALT(LGUI(KC_8)));
    }
}

void dance_7_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[7].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_8))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_8)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_8))); break;
    }
    dance_state[7].step = 0;
}
void on_dance_8(tap_dance_state_t *state, void *user_data);
void dance_8_finished(tap_dance_state_t *state, void *user_data);
void dance_8_reset(tap_dance_state_t *state, void *user_data);

void on_dance_8(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_9)));
        tap_code16(LALT(LGUI(KC_9)));
        tap_code16(LALT(LGUI(KC_9)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_9)));
    }
}

void dance_8_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[8].step = dance_step(state);
    switch (dance_state[8].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_9))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_9)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_9))); register_code16(LALT(LGUI(KC_9)));
    }
}

void dance_8_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[8].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_9))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_9)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_9))); break;
    }
    dance_state[8].step = 0;
}
void on_dance_9(tap_dance_state_t *state, void *user_data);
void dance_9_finished(tap_dance_state_t *state, void *user_data);
void dance_9_reset(tap_dance_state_t *state, void *user_data);

void on_dance_9(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LGUI(KC_0)));
        tap_code16(LALT(LGUI(KC_0)));
        tap_code16(LALT(LGUI(KC_0)));
    }
    if(state->count > 3) {
        tap_code16(LALT(LGUI(KC_0)));
    }
}

void dance_9_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[9].step = dance_step(state);
    switch (dance_state[9].step) {
        case SINGLE_TAP: register_code16(LALT(LGUI(KC_0))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_0)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LGUI(KC_0))); register_code16(LALT(LGUI(KC_0)));
    }
}

void dance_9_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[9].step) {
        case SINGLE_TAP: unregister_code16(LALT(LGUI(KC_0))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_0)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LGUI(KC_0))); break;
    }
    dance_state[9].step = 0;
}
void on_dance_10(tap_dance_state_t *state, void *user_data);
void dance_10_finished(tap_dance_state_t *state, void *user_data);
void dance_10_reset(tap_dance_state_t *state, void *user_data);

void on_dance_10(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RCTL(RGUI(KC_J)));
        tap_code16(RCTL(RGUI(KC_J)));
        tap_code16(RCTL(RGUI(KC_J)));
    }
    if(state->count > 3) {
        tap_code16(RCTL(RGUI(KC_J)));
    }
}

void dance_10_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[10].step = dance_step(state);
    switch (dance_state[10].step) {
        case SINGLE_TAP: register_code16(RCTL(RGUI(KC_J))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_J)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RCTL(RGUI(KC_J))); register_code16(RCTL(RGUI(KC_J)));
    }
}

void dance_10_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[10].step) {
        case SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_J))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_J)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_J))); break;
    }
    dance_state[10].step = 0;
}
void on_dance_11(tap_dance_state_t *state, void *user_data);
void dance_11_finished(tap_dance_state_t *state, void *user_data);
void dance_11_reset(tap_dance_state_t *state, void *user_data);

void on_dance_11(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RCTL(RGUI(KC_L)));
        tap_code16(RCTL(RGUI(KC_L)));
        tap_code16(RCTL(RGUI(KC_L)));
    }
    if(state->count > 3) {
        tap_code16(RCTL(RGUI(KC_L)));
    }
}

void dance_11_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[11].step = dance_step(state);
    switch (dance_state[11].step) {
        case SINGLE_TAP: register_code16(RCTL(RGUI(KC_L))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_L)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RCTL(RGUI(KC_L))); register_code16(RCTL(RGUI(KC_L)));
    }
}

void dance_11_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[11].step) {
        case SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_L))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_L)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_L))); break;
    }
    dance_state[11].step = 0;
}
void on_dance_12(tap_dance_state_t *state, void *user_data);
void dance_12_finished(tap_dance_state_t *state, void *user_data);
void dance_12_reset(tap_dance_state_t *state, void *user_data);

void on_dance_12(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RCTL(RGUI(KC_K)));
        tap_code16(RCTL(RGUI(KC_K)));
        tap_code16(RCTL(RGUI(KC_K)));
    }
    if(state->count > 3) {
        tap_code16(RCTL(RGUI(KC_K)));
    }
}

void dance_12_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[12].step = dance_step(state);
    switch (dance_state[12].step) {
        case SINGLE_TAP: register_code16(RCTL(RGUI(KC_K))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_K)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RCTL(RGUI(KC_K))); register_code16(RCTL(RGUI(KC_K)));
    }
}

void dance_12_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[12].step) {
        case SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_K))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_K)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_K))); break;
    }
    dance_state[12].step = 0;
}
void on_dance_13(tap_dance_state_t *state, void *user_data);
void dance_13_finished(tap_dance_state_t *state, void *user_data);
void dance_13_reset(tap_dance_state_t *state, void *user_data);

void on_dance_13(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RCTL(RGUI(KC_H)));
        tap_code16(RCTL(RGUI(KC_H)));
        tap_code16(RCTL(RGUI(KC_H)));
    }
    if(state->count > 3) {
        tap_code16(RCTL(RGUI(KC_H)));
    }
}

void dance_13_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[13].step = dance_step(state);
    switch (dance_state[13].step) {
        case SINGLE_TAP: register_code16(RCTL(RGUI(KC_H))); break;
        case DOUBLE_TAP: register_code16(LALT(LCTL(LGUI(KC_H)))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RCTL(RGUI(KC_H))); register_code16(RCTL(RGUI(KC_H)));
    }
}

void dance_13_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[13].step) {
        case SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_H))); break;
        case DOUBLE_TAP: unregister_code16(LALT(LCTL(LGUI(KC_H)))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RCTL(RGUI(KC_H))); break;
    }
    dance_state[13].step = 0;
}
void on_dance_14(tap_dance_state_t *state, void *user_data);
void dance_14_finished(tap_dance_state_t *state, void *user_data);
void dance_14_reset(tap_dance_state_t *state, void *user_data);

void on_dance_14(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_SPACE);
        tap_code16(KC_SPACE);
        tap_code16(KC_SPACE);
    }
    if(state->count > 3) {
        tap_code16(KC_SPACE);
    }
}

void dance_14_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[14].step = dance_step(state);
    switch (dance_state[14].step) {
        case SINGLE_TAP: register_code16(KC_SPACE); break;
        case SINGLE_HOLD: register_code16(KC_SPACE); break;
        case DOUBLE_TAP: register_code16(KC_SPACE); register_code16(KC_SPACE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_SPACE); register_code16(KC_SPACE);
    }
}

void dance_14_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[14].step) {
        case SINGLE_TAP: unregister_code16(KC_SPACE); break;
        case SINGLE_HOLD: unregister_code16(KC_SPACE); break;
        case DOUBLE_TAP: unregister_code16(KC_SPACE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_SPACE); break;
    }
    dance_state[14].step = 0;
}
void on_dance_15(tap_dance_state_t *state, void *user_data);
void dance_15_finished(tap_dance_state_t *state, void *user_data);
void dance_15_reset(tap_dance_state_t *state, void *user_data);

void on_dance_15(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_15_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[15].step = dance_step(state);
    switch (dance_state[15].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case SINGLE_HOLD: register_code16(KC_TAB); break;
        case DOUBLE_TAP: register_code16(KC_TAB); register_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_15_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[15].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case SINGLE_HOLD: unregister_code16(KC_TAB); break;
        case DOUBLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[15].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
        [DANCE_7] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_7, dance_7_finished, dance_7_reset),
        [DANCE_8] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_8, dance_8_finished, dance_8_reset),
        [DANCE_9] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_9, dance_9_finished, dance_9_reset),
        [DANCE_10] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_10, dance_10_finished, dance_10_reset),
        [DANCE_11] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_11, dance_11_finished, dance_11_reset),
        [DANCE_12] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_12, dance_12_finished, dance_12_reset),
        [DANCE_13] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_13, dance_13_finished, dance_13_reset),
        [DANCE_14] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_14, dance_14_finished, dance_14_reset),
        [DANCE_15] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_15, dance_15_finished, dance_15_reset),
};
