/*
 * Register map for M-5MOLS 8M Pixel camera sensor with ISP
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd
 * Author: HeungJun Kim, riverful.kim@samsung.com
 *
 * Copyright (C) 2009 Samsung Electronics Co., Ltd
 * Author: Dongsoo Nathaniel Kim, dongsoo45.kim@samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef M5MOLS_REG_H
#define M5MOLS_REG_H

/*
 * Category section register
 *
 * The category means a kind of command set. Including category section,
 * all defined categories in this version supports only, as you see below:
 */
#define CAT_SYSTEM		0x00
#define CAT_PARAM		0x01
#define CAT_MONITOR		0x02
#define CAT_AE			0x03
#define CAT_WB			0x06
#define CAT_FLASH		0x0f	/* related with FW, Verions, booting */

/*
 * Category 0 - System
 *
 * This category supports FW version, managing mode, even interrupt.
 */
#define CAT0_CUSTOMER_CODE	0x00
#define CAT0_PJ_CODE		0x01
#define CAT0_VERSION_FW_H	0x02
#define CAT0_VERSION_FW_L	0x03
#define CAT0_VERSION_HW_H	0x04
#define CAT0_VERSION_HW_L	0x05
#define CAT0_VERSION_PARM_H	0x06
#define CAT0_VERSION_PARM_L	0x07
#define CAT0_VERSION_AWB_H	0x08
#define CAT0_VERSION_AWB_L	0x09
#define CAT0_SYSMODE		0x0b

/*
 * category 1 - Parameter mode
 *
 * This category is dealing with almost camera vendor. In spite of that,
 * It's a register to be able to detailed value for whole camera syste.
 * The key parameter like a resolution, FPS, data interface connecting
 * with Mobile AP, even effects.
 */
#define CAT1_DATA_INTERFACE	0x00
#define CAT1_MONITOR_SIZE	0x01
#define CAT1_MONITOR_FPS	0x02
#define CAT1_EFFECT		0x0b

/*
 * Category 2 - Monitor mode
 *
 * This category supports only monitoring mode. The monitoring mode means,
 * similar to preview. It supports like a YUYV format. At the capture mode,
 * it is handled like a JPEG & RAW formats.
 */
#define CAT2_CFIXB		0x09
#define CAT2_CFIXR		0x0a
#define CAT2_COLOR_EFFECT	0x0b
#define CAT2_CHROMA_LVL		0x0f
#define CAT2_CHROMA_EN		0x10

/*
 * Category 3 - Auto Exposure
 *
 * Currently, it supports only gain value with monitor mode. This device
 * is able to support Shutter, Gain(similar with Aperture), Flicker, at
 * monitor mode & capture mode both.
 */
#define CAT3_AE_LOCK		0x00
#define CAT3_AE_MODE		0x01
#define CAT3_MANUAL_GAIN_MON	0x12
#define CAT3_MAX_GAIN_MON	0x1a

/*
 * Category 6 - White Balance
 *
 * Currently, it supports only auto white balance.
 */
#define CAT6_AWB_LOCK		0x00
#define CAT6_AWB_MODE		0x02

/*
 * Category F - Flash
 *
 * This mode provides functions about internal Flash works and System startup.
 */
#define CATF_CAM_START		0x12	/* It start internal ARM core booting
					 * after power-up */

#endif	/* M5MOLS_REG_H */
