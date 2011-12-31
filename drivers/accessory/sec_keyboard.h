
#ifndef _SEC_KEYBOARD_H_
#define _SEC_KEYBOARD_H_

#include <linux/input.h>

#define KEYBOARD_SIZE   128
#define US_KEYBOARD     0xeb
#define UK_KEYBOARD     0xec

#define KEYBOARD_MIN   0x4
#define KEYBOARD_MAX   0x80

#define MAX_BUF     64

enum KEY_LAYOUT {
	UNKOWN_KEYLAYOUT = 0,
	US_KEYLAYOUT,
	UK_KEYLAYOUT,
};

struct sec_keyboard_drvdata {
	struct input_dev *input_dev;
	struct mutex mutex;
	struct timer_list timer;
	struct timer_list key_timer;
	struct device *keyboard_dev;
	struct work_struct work_msg;
	struct work_struct work_timer;
	struct sec_keyboard_callbacks callbacks;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	void	(*acc_power)(u8 token, bool active);
	void (*check_uart_path)(bool en);
	bool led_on;
	bool dockconnected;
	bool pre_connected;
	bool pressed[KEYBOARD_SIZE];
	bool pre_uart_path;
	int buf_front;
	int buf_rear;
	int acc_int_gpio;
	unsigned int remap_key;
	unsigned int kl;
	unsigned int pre_kl;
	unsigned char key_buf[MAX_BUF+1];
	unsigned short keycode[KEYBOARD_SIZE];
	unsigned long connected_time;
	unsigned long disconnected_time;
};

static const unsigned short sec_keycodes[KEYBOARD_SIZE] = {
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_A,
	KEY_B,
	KEY_C,
	KEY_D,
	KEY_E,
	KEY_F,
	KEY_G,
	KEY_H,
	KEY_I,
	KEY_J,
	KEY_K,
	KEY_L,
	KEY_M,
	KEY_N,
	KEY_O,
	KEY_P,
	KEY_Q,
	KEY_R,
	KEY_S,
	KEY_T,
	KEY_U,
	KEY_V,
	KEY_W,
	KEY_X,
	KEY_Y,
	KEY_Z,
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_6,
	KEY_7,
	KEY_8,
	KEY_9,
	KEY_0,
	KEY_ENTER,
	KEY_BACK,
	KEY_BACKSPACE,
	KEY_TAB,
	KEY_SPACE,
	KEY_MINUS,
	KEY_EQUAL,
	KEY_LEFTBRACE,
	KEY_RIGHTBRACE,
	KEY_HOME,
	KEY_RESERVED,
	KEY_SEMICOLON,
	KEY_APOSTROPHE,
	KEY_GRAVE,
	KEY_COMMA,
	KEY_DOT,
	KEY_SLASH,
	KEY_CAPSLOCK,
	KEY_TIME,
	KEY_F3,
	KEY_WWW,
	KEY_EMAIL,
	KEY_SCREENLOCK,
	KEY_BRIGHTNESSDOWN,
	KEY_BRIGHTNESSUP,
	KEY_MUTE,
	KEY_VOLUMEDOWN,
	KEY_VOLUMEUP,
	KEY_PLAY,
	KEY_REWIND,
	KEY_F15,
	KEY_RESERVED,
	KEY_FASTFORWARD,
	KEY_MENU,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_DELETE,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RIGHT,
	KEY_LEFT,
	KEY_DOWN,
	KEY_UP,
	KEY_NUMLOCK,
	KEY_KPSLASH,
	KEY_APOSTROPHE,
	KEY_KPMINUS,
	KEY_KPPLUS,
	KEY_KPENTER,
	KEY_KP1,
	KEY_KP2,
	KEY_KP3,
	KEY_KP4,
	KEY_KP5,
	KEY_KP6,
	KEY_KP7,
	KEY_KP8,
	KEY_KP9,
	KEY_KPDOT,
	KEY_RESERVED,
	KEY_BACKSLASH,
	KEY_F22,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_HANGEUL,
	KEY_HANJA,
	KEY_LEFTCTRL,
	KEY_LEFTSHIFT,
	KEY_F20,
	KEY_SEARCH,
	KEY_RIGHTALT,
	KEY_RIGHTSHIFT,
	KEY_F21,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_F17,
};

#endif  /*_SEC_KEYBOARD_H_*/
