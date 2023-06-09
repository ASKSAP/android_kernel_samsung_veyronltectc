/*
 * Copyright (C) 2016 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _ET510_LINUX_DIRVER_H_
#define _ET510_LINUX_DIRVER_H_

#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define FEATURE_SPI_WAKELOCK
#endif /* CONFIG_SEC_FACTORY */

#include <linux/module.h>
#include <linux/spi/spi.h>
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spidev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#include <linux/cpufreq.h>
#endif

#include <linux/pinctrl/consumer.h>
#include "../pinctrl/core.h"

/*#define ET510_SPI_DEBUG*/

#ifdef ET510_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...) pr_err(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

#define VENDOR						"EGISTEC"
#define CHIP_ID						"ET510"

/* assigned */
#define ET510_MAJOR					153
/* ... up to 256 */
#define N_SPI_MINORS					32

#define OP_REG_R						0x20
#define OP_REG_R_C						0x22
#define OP_REG_R_C_BW					0x23
#define OP_REG_W						0x24
#define OP_REG_W_C						0x26
#define OP_REG_W_C_BW					0x27
#define OP_NVM_ON_R						0x40
#define OP_NVM_ON_W						0x42
#define OP_NVM_RE						0x44
#define OP_NVM_WE						0x46
#define OP_NVM_OFF						0x48
#define OP_IMG_R						0x50
#define OP_VDM_R						0x60
#define OP_VDM_W						0x62
#define BITS_PER_WORD					8

#define SLOW_BAUD_RATE					12500000

#define DRDY_IRQ_ENABLE					1
#define DRDY_IRQ_DISABLE				0

#define ET510_INT_DETECTION_PERIOD			10
#define ET510_DETECTION_THRESHOLD			10

#define FP_REGISTER_READ				0x01
#define FP_REGISTER_WRITE				0x02
#define FP_GET_ONE_IMG					0x03
#define FP_SENSOR_RESET					0x04
#define FP_POWER_CONTROL				0x05
#define FP_SET_SPI_CLOCK				0x06
#define FP_RESET_SET					0x07
#define FP_REGISTER_BREAD				0x20
#define FP_REGISTER_BWRITE				0x21
#define FP_REGISTER_MREAD				0x22
#define FP_REGISTER_MWRITE				0x23
#define FP_REGISTER_BREAD_BACKWARD		0x24
#define FP_REGISTER_BWRITE_BACKWARD		0x25
#define FP_VDM_READ						0x30
#define FP_VDM_WRITE					0x31
#define FP_NVM_READ						0X40
#define FP_NVM_WRITE					0x41
#define FP_NVM_OFF						0x42
#define FP_NVM_WRITEEX					0x43

#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define FP_DIABLE_SPI_CLOCK				0x10
#define FP_CPU_SPEEDUP					0x11
#define FP_SET_SENSOR_TYPE				0x14
/* Do not use ioctl number 0x15 */
#define FP_SET_LOCKSCREEN				0x16
#define FP_SET_WAKE_UP_SIGNAL				0x17
#endif
#define FP_POWER_CONTROL_ET510				0x18
#define FP_IOCTL_RESERVED_01				0x19

/* trigger signal initial routine */
#define INT_TRIGGER_INIT				0xa4
/* trigger signal close routine */
#define INT_TRIGGER_CLOSE				0xa5
/* read trigger status */
#define INT_TRIGGER_READ				0xa6
/* polling trigger status */
#define INT_TRIGGER_POLLING				0xa7
/* polling abort */
#define INT_TRIGGER_ABORT				0xa8
/* Sensor Registers */
#define FDATA_ET510_ADDR				0x00
#define FSTATUS_ET510_ADDR				0x01
/* Detect Define */
#define FRAME_READY_MASK				0x01

#define SHIFT_BYTE_OF_IMAGE 0
#define DIVISION_OF_IMAGE 4
#define LARGE_SPI_TRANSFER_BUFFER	64
#define MAX_NVM_LEN 32 * 2 /* NVM length in bytes (32 * 16 bits internally)*/
#define NVM_WRITE_LENGTH 4096
#define DETECT_ADM 1

struct egis_ioc_transfer {
	u8 *tx_buf;
	u8 *rx_buf;

	__u32 len;
	__u32 speed_hz;

	__u16 delay_usecs;
	__u8 bits_per_word;
	__u8 cs_change;
	__u8 opcode;
	__u8 pad[3];

};

#define EGIS_IOC_MAGIC			'k'
#define EGIS_MSGSIZE(N) \
	((((N)*(sizeof(struct egis_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof(struct egis_ioc_transfer))) : 0)
#define EGIS_IOC_MESSAGE(N) _IOW(EGIS_IOC_MAGIC, 0, char[EGIS_MSGSIZE(N)])

struct etspi_data {
	dev_t devt;
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 *buf;/* tx buffer for sensor register read/write */
	unsigned bufsiz; /* MAX size of tx and rx buffer */
	unsigned int ocp_en;	/* ocp enable GPIO pin number */
	unsigned int drdyPin;	/* DRDY GPIO pin number */
	unsigned int sleepPin;	/* Sleep GPIO pin number */
	unsigned int ldo_pin;	/* Ldo GPIO pin number */
	unsigned int ldo_pin2;	/* Ldo2 GPIO pin number */
	unsigned int min_cpufreq_limit;

	unsigned int spi_cs;	/* spi cs pin <temporary gpio setting> */

	unsigned int drdy_irq_flag;	/* irq flag */
	bool ldo_onoff;

	/* For polling interrupt */
	int int_count;
	struct timer_list timer;
	struct work_struct work_debug;
	struct workqueue_struct *wq_dbg;
	struct timer_list dbg_timer;
	int sensortype;
#ifdef CONFIG_SENSORS_FINGERPRINT_SYSFS
	struct device *fp_device;
#endif
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	bool enabled_clk;
	bool isGpio_cfgDone;
#ifdef FEATURE_SPI_WAKELOCK
	struct wake_lock fp_spi_lock;
#endif
#endif
	bool tz_mode;
	int detect_period;
	int detect_threshold;
	bool finger_on;
	struct pinctrl *p;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_idle;
};

int etspi_io_burst_read_register(struct etspi_data *etspi,
		struct egis_ioc_transfer *ioc);
int etspi_io_burst_read_register_backward(struct etspi_data *etspi,
		struct egis_ioc_transfer *ioc);
int etspi_io_burst_write_register(struct etspi_data *etspi, 
		struct egis_ioc_transfer *ioc);
int etspi_io_burst_write_register_backward(struct etspi_data *etspi,
		struct egis_ioc_transfer *ioc);
int etspi_io_read_register(struct etspi_data *etspi, u8 *addr, u8 *buf);
int etspi_io_read_registerex(struct etspi_data *etspi, u8 *addr, u8 *buf, u32 len);
int etspi_io_write_register(struct etspi_data *etspi, u8 *buf);
int etspi_read_register(struct etspi_data *etspi, u8 addr, u8 *buf);
int etspi_write_register(struct etspi_data *etspi, u8 addr, u8 buf);
int etspi_io_nvm_read(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_nvm_write(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_nvm_writeex(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_nvm_off(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_vdm_read(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_vdm_write(struct etspi_data *etspi, struct egis_ioc_transfer *ioc);
int etspi_io_get_frame(struct etspi_data *etspi, u8 *frame, u32 size);

#ifdef CONFIG_SENSORS_FINGERPRINT_SYSFS
extern int fingerprint_register(struct device *dev, void *drvdata,
	struct device_attribute *attributes[], char *name);
extern void fingerprint_unregister(struct device *dev,
	struct device_attribute *attributes[]);
#endif

#endif
