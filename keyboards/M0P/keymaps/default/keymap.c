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
  KEY_ACC
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
           _______,    _______,   UN_TRMK,     UN_REST,     UN_COPY,   XXXXXXX,   UN_APRX,   UN_UNEQ,     _______,  _______, \
                                                              _______                                                        \
          ),
        [_ACC] = LAYOUT( /*Numbers and Symbols*/
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
                                                              KEY_ACC                                                        \
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

void keyboard_pre_init_user(void) {
  // Call the keyboard pre init code.

  // Set our LED pins as output
  //setPinOutput(D7);  //Blue (#1 - going L to R)
  //setPinOutput(B5);  //Red (#2)
  //setPinOutput(B6);  //Green #3)
  setPinOutput(B4);  //White (#4)
  //setPinOutput(D5);  //YellowGreen (#5)
  //setPinOutput(C7);  //Yellow (#6)
}




void led_set_user(uint8_t usb_led) {
    if (IS_LED_ON(usb_led, USB_LED_CAPS_LOCK)) {
        writePinHigh(B4);
    } else {
        writePinLow(B4);
    }
}




const uint16_t PROGMEM fn_actions[] = {
    [0] = ACTION_MODS_TAP_KEY(MOD_LSFT, KC_CAPS),
    [1] = ACTION_MODS_TAP_KEY(MOD_RSFT, KC_CAPS)
};
void led_set_keymap(uint8_t usb_led) {
  if (!(usb_led & (1<<USB_LED_NUM_LOCK))) {
    register_code(KC_NUMLOCK);
    unregister_code(KC_NUMLOCK);
  }
}

void matrix_init_user(void) {
  DDRD |= (1 << PD7); //init D7 (Blue)
  PORTD &= ~(1<<PD7); //turn off D7
  DDRB |= (1 << PB5); //init B5 (Red)
  PORTB &= ~(1<<PB1); //turn off B5
  DDRB |= (1 << PB6); //init B6 (Green)
  PORTB &= ~(1<<PB6); //turnoff B6
  DDRD |= (1 << PD5); //init D5 (Yellow Green)
  PORTD &= ~(1<<PD5); //turn off D5
  DDRC |= (1 << PC7); //init C7 (Yellow)
  PORTC &= ~(1<<PC7); //turnoff C7
}

uint32_t layer_state_set_user(uint32_t state)
{

  // if on layer _NAS, turn on D7 LED, otherwise off.
    if (biton32(state) == _NAS || biton32(state) ==_NS2) {
        PORTD |= (1<<PD7);
    } else {
        PORTD &= ~(1<<PD7);
    }


  // if on layer _ACT, turn on B5 LED, otherwise off.
    if (biton32(state) == _ACT) {
        PORTB |= (1<<PB5);
    } else {
        PORTB &= ~(1<<PB5);
    }

  // if on layer _NUM, turn on B5 LED, otherwise off.
    if (biton32(state) == _GUI) {
        PORTB |= (1<<PB6);
    } else {
        PORTB &= ~(1<<PB6);
    }


  return state;
}

//Lock Key Functionality

void BeginLock(void)
{
  lock_active = true;
  lock_searching = true;
  PORTD |= (1<<PD5);
}

void LockKey(uint16_t keycode)
{
  locked_key = keycode;
  register_code(keycode);
  lock_searching = false;
  PORTD &= ~(1<<PD5);
  PORTC |= (1 << PC7);
}
void EndLock(uint16_t keycode)
{
  lock_active = false;
  unregister_code(keycode);
  locked_key = 0;
  PORTD &= ~(1<<PD5);
  PORTC &= ~(1<<PD7);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record)
{
  uprintf("KL: kc: %u, col: %u, row: %u, pressed: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed);

  if (keycode==KEY_ACC)
  {
    /* code */
  }
  else if(keycode == KEY_LCK) // lock key is pressed.
  {
    if(record->event.pressed == 1) //Do nothing special on key down
    {
      return true;
    }
    else //On key up, toggle on/off
    {
      if(lock_active == true) //If lock is active then turn off
      {
        EndLock(locked_key);
        return false;
      }
      else //if lock off, turn on
      {
        BeginLock();
        return false;
      }
    }
  }
  else if(lock_active == true) //Lock is active, need to execute steps that don't involve lock key
  {
    if(lock_searching == true && keycode <0xFF && record->event.pressed == 0 )
    //Lock key pressed, which turned on search.  Key needs to be valid.  On key up, set as locked key
    {
        LockKey(keycode);

        return false;
    }
    else if (lock_searching == false && keycode == locked_key) //if the key has already been set, ignore the button press
    {
      return false;
    }
    //if searching is false or its a quantum key, then continie on.
         //if the locked key has already been set, a normal button push will add to the effect
    else
    {
      return true;
    }
  }
  else //Non-lock related keypresses
  {
    return true;
  }
}
