/*
 * Copyright (C) 2010 Samsung Electronics, Co. Ltd
 *
 * S5P series MIPI CSI slave device support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PLAT_S5P_MIPI_CSIS_H_
#define PLAT_S5P_MIPI_CSIS_H_ __FILE__

struct platform_device;

/**
 * struct s5p_platform_mipi_csis - platform data for MIPI-CSIS
 * @clk_rate: bus clock frequency
 * @lanes: number of data lanes used
 * @alignment: data alignment in bits
 * @hs_settle: HS-RX settle time
 */
struct s5p_platform_mipi_csis {
	unsigned long clk_rate;
	u8 lanes;
	u8 alignment;
	u8 hs_settle;
	int (*phy_enable)(struct platform_device *pdev, bool on);
};

/**
 * struct s5p_csis_phy_control - global MIPI-CSIS PHY control
 * @pdev: platform device the mipi phy state is to be changed for
 * @on: true to enable CSIS PHY and assert its reset,
 * 	false will disable the PHY and put into reset state
 */
int s5p_csis_phy_enable(struct platform_device *pdev, bool on);
extern struct s5p_platform_mipi_csis s5p_mipi_csis0_default_data;
extern struct s5p_platform_mipi_csis s5p_mipi_csis1_default_data;

#endif /* PLAT_S5P_MIPI_CSIS_H_ */
