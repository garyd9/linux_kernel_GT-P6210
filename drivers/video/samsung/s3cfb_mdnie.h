#ifndef __S3CFB_MDNIE_H__
#define __S3CFB_MDNIE_H__

#define S3C_MDNIE_PHY_BASE		0x11CA0000
#define S3C_MDNIE_MAP_SIZE		0x00001000

#define S3C_MDNIE_rR34		0x0088
#define S3C_MDNIE_rR35		0x008C
#define S3C_MDNIE_rR40		0x00A0

#define	S3C_MDNIE_SIZE_MASK		(0x7FF<<0)

#define END_SEQ			0xffff

#define TRUE				1
#define FALSE				0

enum MODE {
	DYNAMIC,
	STANDARD,
	MOVIE,
	MODE_MAX,
};

enum SCENARIO {
	UI_MODE,
	VIDEO_MODE,
	VIDEO_WARM_MODE,
	VIDEO_COLD_MODE,
	CAMERA_MODE,
	NAVI_MODE,
	GALLERY_MODE,
	VT_MODE,
	SCENARIO_MAX,
};

enum OUTDOOR {
	OUTDOOR_OFF,
	OUTDOOR_ON,
	OUTDOOR_MAX,
};

enum TONE {
	TONE_NORMAL,
	TONE_WARM,
	TONE_COLD,
	TONE_MAX,
};

enum CABC {
	CABC_OFF,
#if defined(CONFIG_FB_MDNIE_PWM)
	CABC_ON,
#endif
	CABC_MAX,
};

enum POWER_LUT {
	LUT_DEFAULT,
	LUT_VIDEO,
	LUT_MAX,
};

struct mdnie_tunning_info {
	char *name;
	const unsigned short *seq;
};

struct mdnie_tunning_info_cabc {
	char *name;
	const unsigned short *seq;
	unsigned int idx_lut;
};

struct mdnie_info {
	struct device			*dev;
#if defined(CONFIG_FB_MDNIE_PWM)
	struct lcd_platform_data	*lcd_pd;
	struct backlight_device		*bd;
	unsigned int			bd_enable;
#endif
	struct mutex			lock;

	unsigned int enable;
	enum SCENARIO scenario;
	enum MODE mode;
	enum TONE tone;
	enum OUTDOOR outdoor;
	enum CABC cabc;
	unsigned int tunning;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend    early_suspend;
#endif
};

extern struct mdnie_info *g_mdnie;

int s3c_mdnie_setup(void);
int s3c_mdnie_init_global(struct s3cfb_global *s3cfb_ctrl);
int s3c_mdnie_start(struct s3cfb_global *ctrl);
int s3c_mdnie_off(void);
int s3c_mdnie_stop(void);

extern void init_mdnie_class(void);
extern void mDNIe_Init_Set_Mode(void);

extern int mdnie_send_sequence(struct mdnie_info *mdnie, const unsigned short *seq);
extern int mdnie_txtbuf_to_parsing(char const *pFilepath);

#endif /* __S3CFB_MDNIE_H__ */
