 /************************************************************************************
** File: - fingerprints_hal\drivers\goodix_fp\gf_spi.h
** OPLUS_FEATURE_FINGERPRINT
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      driver definition for sensor driver
**
** Version: 1.0
** Date created: 15:03:11,12/08/2017
** Author:oujinrong@BSP.Fingerprint.Basic
** TAG: BSP.Fingerprint.Basic
**
** --------------------------- Revision History: --------------------------------
** <author>     <data>        <desc>
**  oujinrong   2019/09/03    add power list for Euclid
************************************************************************************/

#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include "../include/oplus_fp_common.h"

/**********************************************************/
enum FP_MODE{
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
};

#define GF_KEY_INPUT_HOME		KEY_HOME
#define GF_KEY_INPUT_MENU		KEY_MENU
#define GF_KEY_INPUT_BACK		KEY_BACK
#define GF_KEY_INPUT_POWER		KEY_POWER
#define GF_KEY_INPUT_CAMERA		KEY_CAMERA

typedef enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAMERA,
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

struct gf_key_map {
	unsigned int type;
	unsigned int code;
};

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

#define GF_IOC_MAGIC    'g'     //define magic number
#define GF_IOC_INIT             _IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_EXIT             _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET            _IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ       _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ      _IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK   _IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK  _IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER     _IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER    _IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT  _IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO      _IOR(GF_IOC_MAGIC, 11, uint8_t)
#define GF_IOC_REMOVE           _IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO        _IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)

#define GF_IOC_WAKELOCK_TIMEOUT_ENABLE        _IO(GF_IOC_MAGIC, 18 )
#define GF_IOC_WAKELOCK_TIMEOUT_DISABLE        _IO(GF_IOC_MAGIC, 19 )
#define GF_IOC_CLEAN_TOUCH_FLAG        _IO(GF_IOC_MAGIC, 20 )

#define  USE_PLATFORM_BUS     1
#define GF_NETLINK_ENABLE 1
#define GF_NET_EVENT_FB_BLACK 2
#define GF_NET_EVENT_FB_UNBLACK 3
#define NETLINK_TEST 25

enum NETLINK_CMD {
    GF_NET_EVENT_TEST = 0,
    GF_NET_EVENT_IRQ = 1,
    GF_NET_EVENT_SCR_OFF,
    GF_NET_EVENT_SCR_ON,
    GF_NET_EVENT_TP_TOUCHDOWN,
    GF_NET_EVENT_TP_TOUCHUP,
    GF_NET_EVENT_UI_READY,
    GF_NET_EVENT_MAX,
};


struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
	struct platform_device *spi;
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
	struct notifier_block notifier;
	char device_available;
	char fb_black;

    /* jinrong add for power */
    unsigned power_num;
    fp_power_info_t pwr_list[FP_MAX_PWR_LIST_LEN];
    uint32_t notify_tpinfo_flag;
    uint32_t ftm_poweroff_flag;
};


static inline int gf_parse_dts(struct gf_dev* gf_dev);
static inline void gf_cleanup(struct gf_dev *gf_dev);

static inline int gf_power_on(struct gf_dev *gf_dev);
static inline int gf_power_off(struct gf_dev *gf_dev);

static inline int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms);
static inline int gf_irq_num(struct gf_dev *gf_dev);

static inline void sendnlmsg(char *msg);
static inline int netlink_init(void);
static inline void netlink_exit(void);

void gf_cleanup_pwr_list(struct gf_dev* gf_dev);
int gf_parse_pwr_list(struct gf_dev* gf_dev);
int gf_parse_ftm_poweroff_flag(struct gf_dev* gf_dev);

#endif /*__GF_SPI_H*/
