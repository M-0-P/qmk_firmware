/* Copyright 2020 M0P
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H


// Defines the keycodes used by our macros in process_record_user
enum my_keycodes {
  KEY_LCK = SAFE_RANGE,
};

enum unicode_names {
    UN_PLCR,
    UN_SECT,
    UN_INFI,
    UN_MICR,
    UN_DEGR,
    UN_CENT,
    UN_DIVI,
    UN_MULT,
    UN_PLMN,
    UN_LSEQ,
    UN_GREQ,
    UN_APRX,
    UN_UNEQ,
    UN_TRMK,
    UN_REST,
    UN_COPY,
    UN_UPEX,
    UN_UPQU,
    UN_ACCA,
    UN_ACCE,
    UN_ACCI,
    UN_ACCO,
    UN_ACCU,
    UN_ACCY,
    UN_TLCN,
    UN_ACLA,
    UN_ACLE,
    UN_ACLI,
    UN_ACLO,
    UN_ACLU,
    UN_ACLY,
    UN_TLLN


};

const uint32_t PROGMEM unicode_map[] = {
  [UN_PLCR] = 0xB6  , // ¶
  [UN_SECT] = 0xA7  , // §
  [UN_INFI] = 0x221E, // ∞
  [UN_MICR] = 0xB5  , // µ
  [UN_DEGR] = 0xBA  , // º
  [UN_CENT] = 0xA2  , // ¢
  [UN_DIVI] = 0xF7  , // ÷
  [UN_MULT] = 0xD7  , // ×
  [UN_PLMN] = 0xB1  , // ±
  [UN_LSEQ] = 0x2264, // ≤
  [UN_GREQ] = 0x2265, // ≥
  [UN_APRX] = 0x2248, // ≈
  [UN_UNEQ] = 0x2260, // ≠
  [UN_TRMK] = 0x2122, // ™
  [UN_REST] = 0xAE  , // ®
  [UN_COPY] = 0xA9  , // ©
  [UN_UPEX] = 0xA1  , // ¡
  [UN_UPQU] = 0xBF  , // ¿
  [UN_ACUA] = 0xC1  , // Á
  [UN_ACUE] = 0xC9  , // É
  [UN_ACUI] = 0xCD  , // Í
  [UN_ACUO] = 0xD3  , // Ó
  [UN_ACUU] = 0xDA  , // Ú
  [UN_ACUY] = 0xDD  , // Ý
  [UN_TLUN] = 0xD1  , // Ñ
  [UN_ACLA] = 0xE1  , // á
  [UN_ACLE] = 0xE9  , // é
  [UN_ACLI] = 0xED  , // í
  [UN_ACLO] = 0xF3  , // ó
  [UN_ACLU] = 0xFA  , // ú
  [UN_ACLY] = 0xFD  , // ý
  [UN_TLLN] = 0xF1    // ñ
};
#define _BAS 0
#define _BAC 1
#define _NAS 2
#define _SM1 3
#define _SM2 4
#define _ACT 5
#define _SET 6

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BAS] = LAYOUT( /*  Base */
      KC_ESC,     S(KC_SCLN),      KC_Q,     KC_W,      KC_B,      KC_M,      KC_U,       KC_Y,        S(KC_1),   KC_BSPC, \
      KC_TAB,        KC_SCLN,      KC_D,     KC_L,      KC_R,      KC_H,      KC_I,       KC_O,     S(KC_SLSH),    KC_ENT, \
     KC_LGUI,       TT(_ACT),      KC_T,     KC_F,      KC_S,      KC_N,      KC_A,       KC_E,       TT(_ACT),   KC_RGUI, \
     KC_LALT,       TT(_NAS),      KC_K,     KC_P,      KC_J,      KC_G,S(KC_QUOT),    KC_QUOT,       TT(_NAS),   KC_RALT, \
    KC_LCTRL,           F(0),      KC_Z,     KC_X,      KC_C,      KC_V,   KC_COMM,     KC_DOT,       TT(_BAC),  KC_RCTRL, \
                                                         KC_SPC                                                            \
    ),
    [_BAS] = LAYOUT( /* CAPS */
        KC_ESC,     S(KC_SCLN),   S(KC_Q),  S(KC_W),   S(KC_B),   S(KC_M),   S(KC_U),    S(KC_Y),        S(KC_1),   KC_BSPC, \
        KC_TAB,        KC_SCLN,   S(KC_D),  S(KC_L),   S(KC_R),   S(KC_H),   S(KC_I),    S(KC_O),     S(KC_SLSH),    KC_ENT, \
       KC_LGUI,       TT(_ACT),   S(KC_T),  S(KC_F),   S(KC_S),   S(KC_N),   S(KC_A),    S(KC_E),       TT(_ACT),   KC_RGUI, \
       KC_LALT,       TT(_NAS),   S(KC_K),  S(KC_P),   S(KC_J),   S(KC_G),S(KC_QUOT),    KC_QUOT,       TT(_NAS),   KC_RALT, \
      KC_LCTRL,       TT(_BAC),   S(KC_Z),  S(KC_X),   S(KC_C),   S(KC_V),   KC_COMM,     KC_DOT,       TT(_BAC),  KC_RCTRL, \
                                                           KC_SPC                                                            \
      ),
      [_NAS] = LAYOUT( /*Numbers and Symbols */
         _______,    S(KC_3),    KC_BSLS,  S(KC_GRV),     S(KC_6),   S(KC_7),   S(KC_4),   S(KC_5),  S(KC_BSLS),  _______, \
         _______,    S(KC_2), S(KC_LBRC), S(KC_RBRC),     KC_SLSH,      KC_7,      KC_8,      KC_9,  S(KC_MINS),  _______, \
         _______,    _______, S(KC_COMM),  S(KC_DOT),     S(KC_8),      KC_4,      KC_5,      KC_6,     _______,  _______, \
         _______,    _______,    S(KC_9),    S(KC_0),     KC_MINS,      KC_1,      KC_2,      KC_3,     _______,  _______, \
         _______,   TT(_SM1),    KC_LBRC,    KC_RBRC,   S(KC_EQL),      KC_0,    KC_EQL,    KC_DOT,    TT(_SM1),  _______, \
                                                            _______                                                        \
        ),
        [_NAS] = LAYOUT( /*Numbers and Symbols */
           _______,    S(KC_3),    KC_BSLS,  S(KC_GRV),     S(KC_6),   S(KC_7),   S(KC_4),   S(KC_5),  S(KC_BSLS),  _______, \
           _______,    S(KC_2), S(KC_LBRC), S(KC_RBRC),     KC_SLSH,      KC_7,      KC_8,      KC_9,  S(KC_MINS),  _______, \
           _______,    _______, S(KC_COMM),  S(KC_DOT),     S(KC_8),      KC_4,      KC_5,      KC_6,     _______,  _______, \
           _______,    _______,    S(KC_9),    S(KC_0),     KC_MINS,      KC_1,      KC_2,      KC_3,     _______,  _______, \
           _______,   TT(_SM1),    KC_LBRC,    KC_RBRC,   S(KC_EQL),      KC_0,    KC_EQL,    KC_DOT,    TT(_SM1),  _______, \
                                                              _______                                                        \
          ),
        [_SM1] = LAYOUT( /*UNICODE (LOWER CASE) */
           _______,    UN_PLCR,   UN_INFI,      KC_GRV,     UN_DEGR,   UN_CENT,   UN_ACLU,   UN_ACLY,     UN_UPEX,  _______, \
           _______,    UN_SECT,   UN_MICR,     XXXXXXX,     UN_DIVI,   XXXXXXX,   UN_ACLI,   UN_ACLO,     UN_UPQU,  _______, \
           _______,    _______,   UN_LSEQ,     UN_GREQ,     UN_MULT,   UN_TILN,   UN_ACLA,   UN_ACLE,     _______,  _______, \
           _______,    _______,   XXXXXXX,     XXXXXXX,     UN_PLMN,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
          TT(_SM2),    _______,   UN_TRMK,     UN_REST,     UN_COPY,   XXXXXXX,   UN_APRX,   UN_UNEQ,     _______, TT(_SM2),\
                                                              _______                                                        \
          ),
          [_SM2] = LAYOUT( /*UNICODE (UPPER CASE) */
             _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   UN_ACCU,   UN_ACCY,     XXXXXXX,  _______, \
             _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   UN_ACCI,   UN_ACCO,     XXXXXXX,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   UN_TLCN,   UN_ACCA,   UN_ACCE,     _______,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
                                                                _______                                                        \
            ),
      [_ACT] = LAYOUT( /* Action - */
           _______,   KEY_LCK,   XXXXXXX,   XXXXXXX,    KC_BRK,   KC_PSCR,    XXXXXXX,  KC_VOLU,    KC_VOLD,   _______, \
           _______,   _______,    KC_INS,   KC_HOME,   KC_PGUP,   XXXXXXX,     KC_UP,   XXXXXXX,    KC_MUTE,   _______, \
           _______,   _______,    KC_DEL,    KC_END,   KC_PGDN,   KC_LEFT,   KC_DOWN,  KC_RIGHT,    _______,   _______, \
           _______,   _______,     KC_F1,     KC_F2,     KC_F3,     KC_F4,     KC_F5,     KC_F6,    _______,   _______, \
           _______,   KC_LSFT,     KC_F7,     KC_F8,     KC_F9,    KC_F10,    KC_F11,    KC_F12,    KC_RSFT,   _______, \
                                                              _______                                                   \
          ),
          /*
          [_SM1] = LAYOUT( Numbers and Symbols
             _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  _______, \
             _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
             _______,    _______,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
                                                                _______                                         BBBBBBB        \
            ),
            */
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case QMKBEST:
            if (record->event.pressed) {
                // when keycode QMKBEST is pressed
                SEND_STRING("QMK is the best thing ever!");
            } else {
                // when keycode QMKBEST is released
            }
            break;
        case QMKURL:
            if (record->event.pressed) {
                // when keycode QMKURL is pressed
                SEND_STRING("https://qmk.fm/" SS_TAP(X_ENTER));
            } else {
                // when keycode QMKURL is released
            }
            break;
    }
    return true;
}

void matrix_init_user(void) {

}

void matrix_scan_user(void) {

}

void led_set_user(uint8_t usb_led) {

}
