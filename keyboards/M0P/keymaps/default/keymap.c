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
  KEY_ACC,
  KY_DOT,
  KY_COMM,
  KY_SCOL,
  KY_COLN,
  KY_QUOT,
  KY_DQOT,
  KY_EXCL,
  KY_QUES
};

bool lock_active = false;  //Is the Lock process active?
bool lock_searching = false; //This is true when the lock starts until a key is held
uint16_t locked_key = 0; //This is the key that is locked.



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
    UN_ACUA,
    UN_ACUE,
    UN_ACUI,
    UN_ACUO,
    UN_ACUU,
    UN_ACUY,
    UN_TLUN,
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
      KC_ESC,        KY_COLN,      KC_Q,     KC_W,      KC_B,      KC_M,      KC_U,       KC_Y,        KY_EXCL,   KC_BSPC, \
      KC_TAB,        KY_SCOL,      KC_D,     KC_L,      KC_R,      KC_H,      KC_I,       KC_O,        KY_QUES,    KC_ENT, \
     KC_LGUI,       TT(_ACT),      KC_T,     KC_F,      KC_S,      KC_N,      KC_A,       KC_E,       TT(_ACT),   KC_RGUI, \
     KC_LALT,       TT(_NAS),      KC_K,     KC_P,      KC_J,      KC_G,   KY_DQOT,    KY_QUOT,       TT(_NAS),   KC_RALT, \
    KC_LCTRL,           F(0),      KC_Z,     KC_X,      KC_C,      KC_V,   KY_COMM,     KY_DOT,           F(1),  KC_RCTRL, \
                                                         KC_SPC                                                            \
    ),
    /*
    [_BAC] = LAYOUT( // CAPS
        KC_ESC,     S(KC_SCLN),   S(KC_Q),  S(KC_W),   S(KC_B),   S(KC_M),   S(KC_U),    S(KC_Y),        S(KC_1),   KC_BSPC, \
        KC_TAB,        KC_SCLN,   S(KC_D),  S(KC_L),   S(KC_R),   S(KC_H),   S(KC_I),    S(KC_O),     S(KC_SLSH),    KC_ENT, \
       KC_LGUI,        _______,   S(KC_T),  S(KC_F),   S(KC_S),   S(KC_N),   S(KC_A),    S(KC_E),       _______,   KC_RGUI, \
       KC_LALT,        _______,   S(KC_K),  S(KC_P),   S(KC_J),   S(KC_G),S(KC_QUOT),    KC_QUOT,       _______,   KC_RALT, \
      KC_LCTRL,        _______,   S(KC_Z),  S(KC_X),   S(KC_C),   S(KC_V),   KC_COMM,     KC_DOT,       _______,  KC_RCTRL, \
                                                           KC_SPC                                                            \
      ),
      */
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
           _______,   TT(_SM1),    KC_LBRC,    KC_RBRC,   S(KC_EQL),      KC_0,    KC_EQL,   KC_PDOT,    TT(_SM1),  _______, \
                                                              _______                                                        \
          ),
        [_SM1] = LAYOUT( /*UNICODE (LOWER CASE) */
           _______,    UN_PLCR,   UN_INFI,      KC_GRV,     UN_DEGR,   UN_CENT,   UN_ACLU,   UN_ACLY,     UN_UPEX,  _______, \
           _______,    UN_SECT,   UN_MICR,     XXXXXXX,     UN_DIVI,   XXXXXXX,   UN_ACLI,   UN_ACLO,     UN_UPQU,  _______, \
           _______,    _______,   UN_LSEQ,     UN_GREQ,     UN_MULT,   UN_TLLN,   UN_ACLA,   UN_ACLE,     _______,  _______, \
           _______,    _______,   XXXXXXX,     XXXXXXX,     UN_PLMN,   XXXXXXX,   XXXXXXX,   XXXXXXX,     _______,  _______, \
          MO(_SM2),    _______,   UN_TRMK,     UN_REST,     UN_COPY,   XXXXXXX,   UN_APRX,   UN_UNEQ,     _______,  MO(_SM2), \
                                                              _______                                                        \
          ),
        [_SM2] = LAYOUT( /*Numbers and Symbols*/
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   UN_ACUU,   UN_ACUY,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   UN_ACUI,   UN_ACUO,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   UN_TLUN,   UN_ACUA,   UN_ACUE,     XXXXXXX,  XXXXXXX, \
           XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
           _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  _______, \
                                                              XXXXXXX                                                        \
          ),
      [_ACT] = LAYOUT( /* Action - */
           _______,   KEY_LCK,   MO(_SET),  XXXXXXX,    KC_BRK,   KC_PSCR,    XXXXXXX,  KC_VOLU,    KC_VOLD,   _______, \
           _______,   XXXXXXX,    KC_INS,   KC_HOME,   KC_PGUP,   XXXXXXX,     KC_UP,   XXXXXXX,    KC_MUTE,   _______, \
           _______,   _______,    KC_DEL,    KC_END,   KC_PGDN,   KC_LEFT,   KC_DOWN,  KC_RIGHT,    _______,   _______, \
           _______,   _______,     KC_F1,     KC_F2,     KC_F3,     KC_F4,     KC_F5,     KC_F6,    _______,   _______, \
           _______,   KC_LSFT,     KC_F7,     KC_F8,     KC_F9,    KC_F10,    KC_F11,    KC_F12,    KC_RSFT,   _______, \
                                                              _______                                                   \
          ),
      [_SET] = LAYOUT( /*Numbers and Symbols*/
         XXXXXXX,    XXXXXXX,   XXXXXXX,       RESET,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
         XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
         XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
         XXXXXXX,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  XXXXXXX, \
         _______,    XXXXXXX,   XXXXXXX,     XXXXXXX,     XXXXXXX,   XXXXXXX,   XXXXXXX,   XXXXXXX,     XXXXXXX,  _______, \
                                                            XXXXXXX                                                        \
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
  //setPinOutput(B4);  //White (#4)
  //setPinOutput(D5);  //YellowGreen (#5)
  //setPinOutput(C7);  //Yellow (#6)
}




void led_set_user(uint8_t usb_led) {
    if (IS_LED_ON(usb_led, USB_LED_CAPS_LOCK)) {
        writePinHigh(B2);
        writePinHigh(C6);
    } else {
        writePinLow(B2);
        writePinLow(C6);
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
  DDRD |= (1 << PD5); //init D5 (Blue)
  PORTD &= ~(1<<PD5); //turn off D5
  DDRC |= (1 << PC7); //init C7 (Red)
  PORTC &= ~(1<<PC7); //turn off FC7
  DDRC |= (1 << PC6); //init C6 (White)
  PORTC &= ~(1<<PC6); //turnoff C6
  DDRB |= (1 << PB2); //init B2 (White)
  PORTB &= ~(1<<PB2); //turn off B2
  DDRB |= (1 << PB1); //init B1 (Green)
  PORTB &= ~(1<<PB1); //turnoff B1
  DDRB |= (1 << PB0); //init B0 (Yellow)
  PORTB &= ~(1<<PB0); //turnoff B0
}

uint32_t layer_state_set_user(uint32_t state)
{

/*
//White Lights
    if (biton32(state) == _BAC || biton32(state) ==_SM2) {
        PORTC |= (1<<PC6);
        PORTB |= (1<<PB2);
    } else {
        PORTC &= ~(1<<PC6);
        PORTB &= ~(1<<PB2);
    }
*/
  //Blue Light
    if (biton32(state) == _NAS || biton32(state) == _SM1 || biton32(state) == _SM2) {
        PORTD |= (1<<PD5);
    } else {
        PORTD &= ~(1<<PD5);
    }


  //Red Light
    if (biton32(state) == _ACT) {
        PORTC |= (1<<PC7);
    } else {
        PORTC &= ~(1<<PC7);
    }

  //Green Light.
    if (biton32(state) == _SM1 || biton32(state) ==_SM2) {
        PORTB |= (1<<PB1);
    } else {
        PORTB &= ~(1<<PB1);

    }


  return state;
}

//Lock Key Functionality

void BeginLock(void)
{
  lock_active = true;
  lock_searching = true;
  PORTB |= (1<<PB1);
}

void LockKey(uint16_t keycode)
{
  locked_key = keycode;
  register_code(keycode);
  lock_searching = false;
  PORTB &= ~(1<<PB1);
  PORTB |= (1 << PB0);
}
void EndLock(uint16_t keycode)
{
  lock_active = false;
  unregister_code(keycode);
  locked_key = 0;
  PORTB &= ~(1<<PB1);
  PORTB &= ~(1<<PB0);
}

//uint16_t mod_pressed = 0;

bool process_record_user(uint16_t keycode, keyrecord_t *record)
{

  uprintf("KL: kc: %u, col: %u, row: %u, pressed: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed);
return true;
  
  switch (keycode) {
    case KY_DOT:
      if(record->event.pressed)
      {
        if(keyboard_report->mods & MOD_BIT (KC_LSFT))
        {
          unregister_code(KC_LSFT);
          register_code(KC_DOT);
          mod_pressed = KC_LSFT;
          return false;
        }
        else if(keyboard_report->mods & MOD_BIT (KC_RSFT))
        {
          unregister_code(KC_RSFT);
          register_code(KC_DOT);
          mod_pressed = KC_RSFT;
          return false;
        }
        else
        {
          register_code(KC_DOT);
		  return false;
        }
      }
      else
      {
          unregister_code(KC_DOT);
          if (mod_pressed != 0)
          {
            register_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
      }
    case KY_COMM:
      if(record->event.pressed)
      {
        if(keyboard_report->mods & MOD_BIT (KC_LSFT))
        {
          unregister_code(KC_LSFT);
          register_code(KC_COMM);
          mod_pressed = KC_LSFT;
          return false;
        }
        else if(keyboard_report->mods & MOD_BIT (KC_RSFT))
        {
          unregister_code(KC_RSFT);
          register_code(KC_COMM);
          mod_pressed = KC_RSFT;
          return false;
        }
        else
        {
          register_code(KC_COMM);
		  return false;
        }
      }
      else
      {
          unregister_code(KC_COMM);
          if (mod_pressed != 0)
          {
            register_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
        }
    case KY_SCOL:
      if(record->event.pressed)
      {
        if(keyboard_report->mods & MOD_BIT (KC_LSFT))
        {
          unregister_code(KC_LSFT);
          register_code(KC_SCLN);
          mod_pressed = KC_LSFT;
          return false;
        }
        else if(keyboard_report->mods & MOD_BIT (KC_RSFT))
        {
          unregister_code(KC_RSFT);
          register_code(KC_SCLN);
          mod_pressed = KC_RSFT;
          return false;
        }
        else
        {
          register_code(KC_SCLN);
		  return false;
        }
      }
      else
      {
          unregister_code(KC_SCLN);
          if (mod_pressed != 0)
          {
            register_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
        }
    case KY_QUOT:
      if(record->event.pressed)
      {
        if(keyboard_report->mods & MOD_BIT (KC_LSFT))
        {
          unregister_code(KC_LSFT);
          register_code(KC_QUOT);
          mod_pressed = KC_LSFT;
          return false;
        }
        else if(keyboard_report->mods & MOD_BIT (KC_RSFT))
        {
          unregister_code(KC_RSFT);
          register_code(KC_QUOT);
          mod_pressed = KC_RSFT;
          return false;
        }
        else
        {
          register_code(KC_QUOT);
		  return false;
        }
      }
      else
      {
          unregister_code(KC_QUOT);
          if (mod_pressed != 0)
          {
            register_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
        }
    case KY_DQOT:
      if(record->event.pressed)
      {
        if((keyboard_report->mods & MOD_BIT (KC_LSFT)) || (keyboard_report->mods & MOD_BIT (KC_RSFT)))
        {
          register_code(KC_QUOT);
          return false;
        }
        else
        {
		  register_code(KC_LSFT);
          register_code(KC_QUOT);
		  mod_pressed = KC_LSFT;
		  return false; 
        }
      }
      else
      {
          unregister_code(KC_QUOT);
          if (mod_pressed != 0)
          {
            unregister_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
      }
	case KY_SCOL:
      if(record->event.pressed)
      {
        if((keyboard_report->mods & MOD_BIT (KC_LSFT)) || (keyboard_report->mods & MOD_BIT (KC_RSFT)))
        {
          register_code(KC_SCLN);
          return false;
        }
        else
        {
		  register_code(KC_LSFT);
          register_code(KC_SCLN);
		  mod_pressed = KC_LSFT;
		  return false; 
        }
      }
      else
      {
          unregister_code(KC_SCLN);
          if (mod_pressed != 0)
          {
            unregister_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
      }	
	case KY_EXCL:
      if(record->event.pressed)
      {
        if((keyboard_report->mods & MOD_BIT (KC_LSFT)) || (keyboard_report->mods & MOD_BIT (KC_RSFT)))
        {
          register_code(KC_1);
          return false;
        }
        else
        {
		  register_code(KC_LSFT);
          register_code(KC_1);
		  mod_pressed = KC_LSFT;
		  return false; 
        }
      }
      else
      {
          unregister_code(KC_1);
          if (mod_pressed != 0)
          {
            unregister_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
      }	
	  	  
	case KY_QUES:
      if(record->event.pressed)
      {
        if((keyboard_report->mods & MOD_BIT (KC_LSFT)) || (keyboard_report->mods & MOD_BIT (KC_RSFT)))
        {
          register_code(KC_SLSH);
          return false;
        }
        else
        {
		  register_code(KC_LSFT);
          register_code(KC_SLSH);
		  mod_pressed = KC_LSFT;
		  return false; 
        }
      }
      else
      {
          unregister_code(KC_SLSH);
          if (mod_pressed != 0)
          {
            unregister_code(mod_pressed);
            mod_pressed = 0;
          }
		  return false;
      }
	case KEY_LCK:
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
    default:
		if(lock_active == true) //Lock is active, need to execute steps that don't involve lock key
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

/*
  if(keycode == KEY_LCK) // lock key is pressed.
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
*/
}
