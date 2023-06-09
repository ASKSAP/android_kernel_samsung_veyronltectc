#ifndef _HRM_MAX86902_H_
#define _HRM_MAX86902_H_

int max869_i2c_read(u32 reg, u32 *value, u32 *size);
int max869_i2c_write(u32 reg, u32 value);
int max869_init_device(struct i2c_client *client);
int max869_deinit_device(void);
int max869_enable(enum hrm_mode mode);
int max869_disable(enum hrm_mode mode);
int max869_get_current(u8 *d1, u8 *d2, u8 *d3, u8 *d4);
int max869_set_current(u8 d1, u8 d2, u8 d3, u8 d4);
int max869_read_data(struct hrm_output_data *data);
int max869_get_chipid(u64 *chip_id);
int max869_get_name_chipset(char *name);
int max869_get_name_vendor(char *name);
int max869_get_threshold(s32 *threshold);
int max869_set_threshold(s32 threshold);
int max869_set_eol_enable(u8 enable);
int max869_get_eol_result(char *result);
int max869_get_eol_status(u8 *status);
int max869_debug_set(u8 mode);
int max869_get_fac_cmd(char *cmd_result);

#endif /* _HRM_MAX86902_H_ */
