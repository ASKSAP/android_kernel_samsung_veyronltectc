/*
 * =================================================================
 *
 *
 *	Description:  samsung display panel file
 *
 *	Author: jb09.kim
 *	Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2012, Samsung Electronics. All rights reserved.

*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#include "ss_dsi_smart_dimming_S6E3FA5_AMS420MS01.h"

//#define SMART_DIMMING_DEBUG

static char max_lux_table[GAMMA_SET_MAX];

/*
 * PRINT LOG
 */
static void print_RGB_offset(struct SMART_DIM *pSmart)
{
	int i;

	for (i=0; i<V_MAX; i++) {
		pr_err("%s MTP_OFFSET %4s %3d %3d %3d \n", __func__, V_LIST[i],
			pSmart->MTP[i][R],
			pSmart->MTP[i][G],
			pSmart->MTP[i][B]);
	}
}

static void print_lux_table(struct SMART_DIM *psmart, char* type)
{
	int lux_loop;
	int cnt;
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);

	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
			if (!strcmp(type,"DEC"))
				snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %3d",
					psmart->gen_table[lux_loop].gamma_setting[cnt]);
			else
				snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %02x",
					psmart->gen_table[lux_loop].gamma_setting[cnt]);

		pr_err("lux[%3d]  %s\n", psmart->plux_table[lux_loop], pBuffer);
		memset(pBuffer, 0x00, 256);
	}
}

static void print_hbm_lux_table(struct SMART_DIM *psmart, char* type)
{
	int i,j;
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);

	for (i = 0; i < HBM_INTERPOLATION_STEP; i++) {
		for (j = 0; j < GAMMA_SET_MAX; j++) {
			if (!strcmp(type,"DEC"))
				snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %3d",
						psmart->hbm_interpolation_table[i].gamma_setting[j]);
			else
				snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %02x",
					psmart->hbm_interpolation_table[i].gamma_setting[j]);
		}
		pr_err("hbm[%3d]  %s\n", hbm_interpolation_candela_table[i], pBuffer);
		memset(pBuffer, 0x00, 256);
	}
}

static void print_aid_log(struct smartdim_conf *conf)
{
	print_RGB_offset(conf->psmart);
	print_lux_table(conf->psmart, "DEC");
	print_hbm_lux_table(conf->psmart, "DEC");

	print_lux_table(conf->psmart, "HEX");
	print_hbm_lux_table(conf->psmart, "HEX");
}

// 1. VT, V255 : VREG1 - VREG1 * (OutputGamma + numerator_TP)/(denominator_TP)
static unsigned long long v255_TP_gamma_voltage_calc(int VREG1, int OutputGamma,
								int fraction[])
{
	unsigned long long val1, val2, val3;

	val1 = (unsigned long long)(fraction[0] + OutputGamma) << BIT_SHIFT;
	do_div(val1, fraction[1]);
	val2 = (VREG1 * val1) >> BIT_SHIFT;

	val3 = VREG1 - val2;

	return val3;
}

// 2. VT1, VT7 : VREG1 - (VREG1 - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP)
// 3. other VT :    VT - (   VT - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP)
static unsigned long long other_TP_gamma_voltage_calc(int VT, int V_nextTP, int OutputGamma,
								int fraction[])
{
	unsigned long long val1, val2, val3, val4;

	val1 = VT - V_nextTP;

	val2 = (unsigned long long)(fraction[0] + OutputGamma) << BIT_SHIFT;
	do_div(val2, fraction[1]);
	val3 = (val1 * val2) >> BIT_SHIFT;

	val4 = VT - val3;

	return val4;
}

/*
 *	each TP's gamma voltage calculation (2.3.3)
 */
static void TP_gamma_voltage_calc(struct SMART_DIM *pSmart)
{
	int i,j;
	int OutputGamma;
	int MTP_OFFSET;

	/* VT first */
	for (j = 0; j < RGB_MAX; j++) {
		MTP_OFFSET = pSmart->MTP[VT][j];
		fraction[VT][0] = vt_coefficient[MTP_OFFSET];
		pSmart->RGB_OUTPUT[VT][j] =
			v255_TP_gamma_voltage_calc(pSmart->vregout_voltage,
										0,
										fraction[VT]);
	}

	/* V255 */
	for (j = 0; j < RGB_MAX; j++) {
		MTP_OFFSET = pSmart->MTP[V255][j];
		OutputGamma = MTP_OFFSET + center_gamma[V255][j];
		pSmart->RGB_OUTPUT[V255][j] =
			v255_TP_gamma_voltage_calc(pSmart->vregout_voltage,
										OutputGamma,
										fraction[V255]);
	}

	/* V203 ~ V11 */
	for (i = V203; i >= V11; i--) {
		for (j = 0; j < RGB_MAX; j++) {
			MTP_OFFSET = pSmart->MTP[i][j];
			OutputGamma = MTP_OFFSET + center_gamma[i][j];
			pSmart->RGB_OUTPUT[i][j] =
				other_TP_gamma_voltage_calc(pSmart->RGB_OUTPUT[VT][j],
											pSmart->RGB_OUTPUT[i+1][j],
											OutputGamma,
											fraction[i]);
		}
	}

	/* V7, V1*/
	for (i = V7; i >= V1; i--) {
		for (j = 0; j < RGB_MAX; j++) {
			MTP_OFFSET = pSmart->MTP[i][j];
			OutputGamma = MTP_OFFSET + center_gamma[i][j];
			pSmart->RGB_OUTPUT[i][j] =
				other_TP_gamma_voltage_calc(pSmart->vregout_voltage,
											pSmart->RGB_OUTPUT[i+1][j],
											OutputGamma,
											fraction[i]);
		}
	}

#ifdef SMART_DIMMING_DEBUG
	for (i = 0; i < V_MAX; i++) {
		pr_err("%16s %4s %d %d %d\n", __func__,V_LIST[i],
				pSmart->RGB_OUTPUT[i][R],
				pSmart->RGB_OUTPUT[i][G],
				pSmart->RGB_OUTPUT[i][B]);
	}
#endif

}

// 1. V255 : ((VREG1 - V255) * denominator_TP / VREG) - numerator_TP
static unsigned long long v255_TP_gamma_code_calc(int VREG, int GRAY, int fraction[])
{
	unsigned long long val1, val2, val3;

	val1 = VREG - GRAY;
	val2 = val1 * fraction[1];
	do_div(val2, VREG);
	val3 = val2 - fraction[0];

	return val3;
}

// 2. other : (VT    - V_TP)* denominator_TP /(VT    - V_nextTP) - numerator_TP
// 3. V7,V1 : (VREG1 - V_TP)* denominator_TP /(VREG1 - V_nextTP) - numerator_TP
static unsigned long long other_TP_gamma_code_calc(int VT, int GRAY, int nextGRAY, int fraction[])
{
	unsigned long long val1, val2, val3, val4;

	val1 = VT - GRAY;
	val2 = val1 * fraction[1];
	val3 = VT - nextGRAY;
	do_div(val2, val3);
	val4 = val2 - fraction[0];

	return val4;
}

/*
 *	each TP's gamma code calculation (3.7.1)
 */
static void TP_gamma_code_calc(struct SMART_DIM *pSmart, int* M_GRAY, int *str)
{
	unsigned long long val;
	int TP, nextTP;
	int i,j;
	int cnt = 0; // str[0] ~ str[34]

	/* V255 */
	TP = M_GRAY[V255];
	for (j = 0; j < RGB_MAX; j++) {
		val = v255_TP_gamma_code_calc(pSmart->vregout_voltage,
						     		pSmart->GRAY[TP][j],
									fraction[V255]);
		str[cnt++] = (val & 0xFF00) >> 8;
		str[cnt++] = val & 0xFF;
	}

	/* V203 ~ V11 */
	for (i = V203; i >= V11; i--) {
		for (j = 0; j < RGB_MAX; j++) {
			TP = M_GRAY[i];
			nextTP = M_GRAY[i+1];
			val = other_TP_gamma_code_calc(pSmart->RGB_OUTPUT[VT][j],
										pSmart->GRAY[TP][j],
										pSmart->GRAY[nextTP][j],
										fraction[i]);
			str[cnt++] = val;
		}
	}

	/* V7, V1 */
	for (i = V7; i >= V1; i--) {
		for (j = 0; j < RGB_MAX; j++) {
			TP = M_GRAY[i];
			nextTP = M_GRAY[i+1];
			val = other_TP_gamma_code_calc(pSmart->vregout_voltage,
										pSmart->GRAY[TP][j],
										pSmart->GRAY[nextTP][j],
										fraction[i]);
			str[cnt++] = val;
		}
	}

	/* VT */
	str[cnt++] = center_gamma[VT][G];
	str[cnt++] = center_gamma[VT][B];
}

/* gray scale = V_down + (V_up - V_down) * num / den */
static int gray_scale_calc(int v_up, int v_down, int num, int den)
{
	unsigned long long val1, val2;

	val1 = v_up - v_down;

	val2 = (unsigned long long)(val1 * num) << BIT_SHIFT;

	do_div(val2, den);

	val2 >>= BIT_SHIFT;

	val2 += v_down;

	return (int)val2;
}

static int generate_gray_scale(struct SMART_DIM *pSmart)
{
	int i, V_idx = 0;
	int cnt = 0, cal_cnt = 0;
	int den = 0;
	/*
		GRAY OUTPUT VOLTAGE of TP's (V0,V1,V7,V11,V23,V35,V51,V87,V151,V203,V255)
		(V0 is VREG1)
	*/
	for (i = 0; i < RGB_MAX; i++)
		pSmart->GRAY[0][i] = pSmart->vregout_voltage;

	for (cnt = 1; cnt < V_MAX; cnt++) {
		for (i = 0; i < RGB_MAX; i++) {
			pSmart->GRAY[INFLECTION_VOLTAGE_ARRAY[cnt]][i] =
				pSmart->RGB_OUTPUT[cnt][i];
		}
	}

	/*
		ALL GRAY OUTPUT VOLTAGE (0~255)
	*/
	for (cnt = 0; cnt < GRAY_SCALE_MAX; cnt++) {
		if (cnt == INFLECTION_VOLTAGE_ARRAY[V_idx]) {
			cal_cnt = 1;
			V_idx++;
		} else {
			den = INFLECTION_VOLTAGE_ARRAY[V_idx] - INFLECTION_VOLTAGE_ARRAY[V_idx-1];

			for (i=0; i< RGB_MAX; i++)
				pSmart->GRAY[cnt][i] = gray_scale_calc(
					pSmart->GRAY[INFLECTION_VOLTAGE_ARRAY[V_idx-1]][i],
					pSmart->GRAY[INFLECTION_VOLTAGE_ARRAY[V_idx]][i],
					den - cal_cnt, den);
			cal_cnt++;
		}
	}

#ifdef SMART_DIMMING_DEBUG
	for (cnt = 0; cnt < GRAY_SCALE_MAX; cnt++) {
		pr_err("%s %8d %8d %8d %d\n", __func__,
			pSmart->GRAY[cnt][R],
			pSmart->GRAY[cnt][G],
			pSmart->GRAY[cnt][B], cnt);
	}
#endif

	return 0;
}

static char offset_cal(int offset,  int value)
{
	if (value - offset < 0)
		return 0;
	else if (value - offset > 255)
		return 0xFF;
	else
		return value - offset;
}

/* subtration MTP_OFFSET value from generated gamma table */
static void mtp_offset_substraction(struct SMART_DIM *pSmart, int *str)
{
	int level_255_temp = 0;
	int level_255_temp_MSB = 0;
	int MTP_V255;
	int i,j;
	int idx = 0;

	/* V255 : str[0] ~ str[5] */
	for (j = 0; j < RGB_MAX; j++) {
		level_255_temp = (str[idx] << 8) | str[idx+1] ;
		MTP_V255 = pSmart->MTP[V255][j];
		level_255_temp -=  MTP_V255;
		level_255_temp_MSB = level_255_temp / 256;

		str[idx] = level_255_temp_MSB & 0xff;
		str[idx+1] = level_255_temp & 0xff;

		idx += 2;
	}

	/* V203 ~ V7 : str[6] ~ str[29] */
	for (i = V203; i >= V1; i--) {
		for (j = 0; j < RGB_MAX; j++) {
			str[idx] = offset_cal(pSmart->MTP[i][j], str[idx]);
			idx++;
		}
	}
}

/* 3.6 - TP's Voltage Search */
static int searching_function(long long candela, int *index, int gamma_curve)
{
	long long delta_1 = 0, delta_2 = 0;
	int cnt;

	/*
	*	This searching_functin should be changed with improved
		searcing algorithm to reduce searching time.
	*/
	*index = -1;

	for (cnt = 0; cnt < (GRAY_SCALE_MAX-1); cnt++) {
		if (gamma_curve == GAMMA_CURVE_1P9) {
			delta_1 = candela - curve_1p9_360[cnt];
			delta_2 = candela - curve_1p9_360[cnt+1];
		} else if (gamma_curve == GAMMA_CURVE_2P15) {
			delta_1 = candela - curve_2p15_360[cnt];
			delta_2 = candela - curve_2p15_360[cnt+1];
		} else if (gamma_curve == GAMMA_CURVE_2P2) {
			delta_1 = candela - curve_2p2_420[cnt];
			delta_2 = candela - curve_2p2_420[cnt+1];
		} else {
			delta_1 = candela - curve_2p2_360[cnt];
			delta_2 = candela - curve_2p2_360[cnt+1];
		}

		if (delta_2 < 0) {
			*index = (delta_1 + delta_2) <= 0 ? cnt : cnt+1;
			break;
		}

		if (delta_1 == 0) {
			*index = cnt;
			break;
		}

		if (delta_2 == 0) {
			*index = cnt+1;
			break;
		}
	}

	if (*index == -1)
		return -EINVAL;
	else
		return 0;
}

static int get_max_candela(void)
{
	return 420;
}

static int get_vreg_voltage(void)
{
	return VREG0_REF_6P9;
}

static int get_base_luminance(struct SMART_DIM *pSmart)
{
	int cnt;
	int base_luminance[LUMINANCE_MAX][2];

	if (id3 == 0x00)
		memcpy(base_luminance, base_luminance_revA, sizeof(base_luminance_revA));
	else // (id3, 0x01: revC;  0x02: revD, same AID sheet)
		memcpy(base_luminance, base_luminance_revC, sizeof(base_luminance_revC));

	for (cnt = 0; cnt < LUMINANCE_MAX; cnt++)
		if (base_luminance[cnt][0] == pSmart->brightness_level)
			return base_luminance[cnt][1];

	return -1;
}

static int get_gamma_curve(struct SMART_DIM *pSmart)
{
	return GAMMA_CURVE_2P2;
}

static int get_gradation_offset(int table_index, int index)
{
	if (id3 == 0x00)
		return gradation_offset_revA[table_index][index];
	else // (id3, 0x01: revC;  0x02: revD, same AID sheet)
		return gradation_offset_revC[table_index][index];
}

static int get_rgb_offset(int table_index, int index)
{
	if (id3 == 0x00)
		return rgb_offset_revA[table_index][index];
	else // (id3, 0x01: revC;  0x02: revD, same AID sheet)
		return rgb_offset_revC[table_index][index];
}

static void TP_L_calc(struct SMART_DIM *pSmart, long long *L, int base_level)
{
	long long temp_cal_data = 0;
	int point_index;
	int cnt;

	if (pSmart->brightness_level < get_max_candela()) {
		for (cnt = 0; cnt < V_MAX; cnt++) {
			point_index = INFLECTION_VOLTAGE_ARRAY[cnt];
			temp_cal_data =
				((long long)(candela_coeff_2p15[point_index])) *
				((long long)(base_level));

			L[cnt] = temp_cal_data;
		}

	} else {
		for (cnt = 0; cnt < V_MAX; cnt++) {
			point_index = INFLECTION_VOLTAGE_ARRAY[cnt];
			temp_cal_data =
				((long long)(candela_coeff_2p2[point_index])) *
				((long long)(base_level));

			L[cnt] = temp_cal_data;
		}
	}
}

static void TP_M_GRAY_calc(struct SMART_DIM *pSmart, long long *L, int *M_GRAY, int table_index)
{
	int i;

	M_GRAY[VT] = 0;
	M_GRAY[V1] = 1;

	for (i = V7; i < V_MAX; i++) {
		/* 3.6 - TP's Voltage Search */
		if (searching_function(L[i],
			&(M_GRAY[i]), get_gamma_curve(pSmart))) {
			pr_err("%s searching functioin error cnt:%d\n",
			__func__, i);
		}
	}

	/* 2.2.6 - add offset for Candela Gamma Compensation (V255~V7) */
	for (i = V255; i >= V7; i--) {
		if (table_index == -1) {
			table_index = LUMINANCE_MAX-1;
			pr_err("%s : fail candela table_index cnt : %d brightness %d\n",
				__func__, i, pSmart->brightness_level);
		}
		M_GRAY[i] += get_gradation_offset(table_index, V255-i);
	}
}

static void Color_shift_compensation(struct SMART_DIM *pSmart, int *gamma_setting, int table_index)
{
	int i;
	int level_255_temp_MSB = 0;
	int level_V255 = 0;

	for (i = 0; i < RGB_COMPENSATION; i++) {
		if (table_index == -1) {
			table_index = LUMINANCE_MAX-1;
			pr_err("%s : fail RGB table_index cnt : %d brightness %d\n",
				__func__, i, pSmart->brightness_level);
		}

		if (i < 3) {
			level_V255 = gamma_setting[i * 2] << 8 | gamma_setting[(i * 2) + 1];
			level_V255 += get_rgb_offset(table_index, i);
			level_255_temp_MSB = level_V255 / 256;

			gamma_setting[i * 2] = level_255_temp_MSB & 0xff;
			gamma_setting[(i * 2) + 1] = level_V255 & 0xff;
		} else {
			gamma_setting[i+3] += get_rgb_offset(table_index, i);
		}
	}
}

static void gamma_init(struct SMART_DIM *pSmart, char *str, int size, int table_index)
{
#ifdef SMART_DIMMING_DEBUG
	int i;
#endif
	long long L[V_MAX] = {0, };
	int M_GRAY[V_MAX] = {0, };
	int gamma_setting[GAMMA_SET_MAX];
	int base_level = 0;
	int cnt;

	pr_debug("%s : start !! table_index(%d)\n",__func__,table_index);

	/* get base luminance */
	base_level = get_base_luminance(pSmart);
	if (base_level < 0)
		pr_err("%s : can not find base luminance!!\n", __func__);

	/* 2.2.5 (F) TP's Luminance  (x4194304) */
	TP_L_calc(pSmart, L, base_level);

	/* 3.5.1 M-Gray */
	TP_M_GRAY_calc(pSmart, L, M_GRAY, table_index);

#ifdef SMART_DIMMING_DEBUG
	pr_err("\n brightness_level (%d) \n %16s %8s\n",
			   pSmart->brightness_level, "L", "M_GRAY");

	for (i=0; i<V_MAX; i++)
		pr_err("%5s %11llu %8d\n", V_LIST[i], L[i], M_GRAY[i]);
#endif

	/* 3.7.1 - Generate Gamma code */
	TP_gamma_code_calc(pSmart, M_GRAY, gamma_setting);

	/* 3.7.2 - Color Shift (RGB compensation) */
	Color_shift_compensation(pSmart, gamma_setting, table_index);

	/* subtration MTP_OFFSET value from generated gamma table */
	mtp_offset_substraction(pSmart, gamma_setting);

	/* To avoid overflow */
	for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
		str[cnt] = gamma_setting[cnt];
}

static void generate_hbm_gamma(struct SMART_DIM *psmart, char *str, int size)
{
#ifdef SMART_DIMMING_DEBUG
	int i;
	char log_buf[256];
#endif

	struct illuminance_table *ptable = (struct illuminance_table *)
						(&(psmart->hbm_interpolation_table));
	/* 443nit(level 6) is not used */
	memcpy(str, &(ptable[psmart->hbm_brightness_level+1].gamma_setting), size);

#ifdef SMART_DIMMING_DEBUG
	memset(log_buf, 0x00, 256);
	for (i = 0; i < GAMMA_SET_MAX; i++)
		snprintf(log_buf + strnlen(log_buf, 256), 256, " %3d",str[i]);

	pr_err("generate_hbm_gamma[%d] : %s\n", psmart->hbm_brightness_level+1, log_buf);
	memset(log_buf, 0x00, 256);
	pr_err("\n");
#endif
}

static void generate_gamma(struct SMART_DIM *psmart, char *str, int size)
{
	int lux_loop;
	struct illuminance_table *ptable = (struct illuminance_table *)
						(&(psmart->gen_table));

	/* searching already generated gamma from table */
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		if (ptable[lux_loop].lux == psmart->brightness_level) {
			memcpy(str, &(ptable[lux_loop].gamma_setting), size);
			break;
		}
	}

	/* searching fail... Setting 300CD value on gamma table */
	if (lux_loop == psmart->lux_table_max) {
		pr_err("%s searching fail lux : %d\n", __func__,
				psmart->brightness_level);
		memcpy(str, max_lux_table, size);
	}

#ifdef SMART_DIMMING_DEBUG
	if (lux_loop != psmart->lux_table_max)
		pr_err("%s searching ok index : %d lux : %d", __func__,
			lux_loop, ptable[lux_loop].lux);
#endif
}

/*
	set max_lux_table for max lux
*/
static void set_max_lux_table(int ldi_revision)
{
	int i,j;

	pr_err("%s ldi_revision: 0x%x", __func__, ldi_revision);

	for (i=0,j=0; i<6; i+=2,j++) {
		max_lux_table[i] = (center_gamma[V255][j] & 0xF00) >> 8;
		max_lux_table[i+1] = center_gamma[V255][j] & 0xFF;
	}

	for (i=6,j=0; i<35; i++,j++) {
		max_lux_table[i] = center_gamma[V_MAX - i/3][j%3];
	}
}

static int char_to_int(char data1)
{
	int cal_data;

	if (data1 & 0x80) {
		cal_data = data1 & 0x7F;
		cal_data *= -1;
	} else
		cal_data = data1;

	return cal_data;
}

/*
	copy psmart->MTP_ORIGN to psmart->MTP
*/
static void mtp_sorting(struct SMART_DIM *psmart)
{
	int i,j;
	char *pfrom;
	int cnt;

	pfrom = psmart->MTP_ORIGN;

	/* V255 */
	cnt = 0;
	for (i = 0; i < RGB_MAX; i++) {
		if (pfrom[i*2])
			psmart->MTP[V255][i] = -1 * pfrom[i*2+1];
		else
			psmart->MTP[V255][i] = pfrom[i*2+1];
	}

	/* V203 ~ V1 */
	cnt = 6;
	for (i = V203; i > VT; i--) {
		for (j=0; j<RGB_MAX; j++) {
			psmart->MTP[i][j] = char_to_int(pfrom[cnt]);
			cnt++;
		}
	}

	/* VT */
	psmart->MTP[VT][R] = char_to_int((pfrom[33] & 0xF0) >> 4);
	psmart->MTP[VT][G] = char_to_int(pfrom[33] & 0xF);
	psmart->MTP[VT][B] = char_to_int(pfrom[34]);
}

#define HBM_CANDELA 600
#define HBM_GAMMA_SET_CNT 33
#define PACKED_GAMM_SET_CNT 30 /* packed V255 R,G.B */
#define V255_RGB_MSB_CNT 3
static void hbm_interpolation_init(struct SMART_DIM *pSmart)
{
	int loop, gamma_index;
	int rate;
	int hbm_gamma[PACKED_GAMM_SET_CNT];
	int max_gamma[PACKED_GAMM_SET_CNT];
	char *hbm_payload;
	int hbm_interpolation_gamma[HBM_INTERPOLATION_STEP][PACKED_GAMM_SET_CNT];
	int normal_max_candela = pSmart->gen_table[LUMINANCE_MAX-1].lux;
	hbm_payload = pSmart->hbm_payload;

	if (!hbm_payload) {
		pr_err("%s : no hbm_payload..\n", __func__);
		return;
	}

	for (loop = 0, gamma_index = 0; gamma_index < HBM_GAMMA_SET_CNT;) {
		/* V255 R,G,B */
		if (gamma_index < 6) {
			hbm_gamma[loop++] = (hbm_payload[gamma_index] << 8) + hbm_payload[gamma_index+1];
			gamma_index += 2;
		} else {
			hbm_gamma[loop++] = hbm_payload[gamma_index];
			gamma_index++;
		}
	}

	for (loop = 0, gamma_index = 0; gamma_index < HBM_GAMMA_SET_CNT;) {
		/* V255 R,G,B */
		if (gamma_index < 6) {
			max_gamma[loop++] = (max_lux_table[gamma_index] << 8) + max_lux_table[gamma_index+1];
			gamma_index += 2;
		} else {
			max_gamma[loop++] = max_lux_table[gamma_index];
			gamma_index++;
		}
	}

	/* generate interpolation hbm gamma */
	for (loop = 0 ; loop < HBM_INTERPOLATION_STEP; loop++) {
		rate = ((hbm_interpolation_candela_table[loop] - normal_max_candela) * BIT_SHFIT_MUL) / (HBM_CANDELA - normal_max_candela);
		for (gamma_index = 0; gamma_index < PACKED_GAMM_SET_CNT; gamma_index++)
			hbm_interpolation_gamma[loop][gamma_index] = max_gamma[gamma_index] +
				((hbm_gamma[gamma_index] - max_gamma[gamma_index]) * rate) / BIT_SHFIT_MUL;
	}

	/* update max hbm gamma with origin hbm gamma */
	for (loop = 0; loop < PACKED_GAMM_SET_CNT; loop++)
		hbm_interpolation_gamma[HBM_INTERPOLATION_STEP-1][loop] = hbm_gamma[loop];

	for (loop = 0; loop < HBM_INTERPOLATION_STEP; loop++) {
		for (gamma_index = 0; gamma_index < PACKED_GAMM_SET_CNT; gamma_index++) {
			/* V255 R,G,B */
			if (gamma_index < 3) {
				pSmart->hbm_interpolation_table[loop].gamma_setting[gamma_index * 2] =
					(hbm_interpolation_gamma[loop][gamma_index] & 0xF00) >> 8;
				pSmart->hbm_interpolation_table[loop].gamma_setting[(gamma_index * 2) + 1] =
					hbm_interpolation_gamma[loop][gamma_index] & 0xFF;
			} else {
				pSmart->hbm_interpolation_table[loop].gamma_setting[gamma_index + V255_RGB_MSB_CNT] =
					hbm_interpolation_gamma[loop][gamma_index];
			}
		}
	}

#ifdef SMART_DIMMING_DEBUG
	print_hbm_lux_table(pSmart, "DEC");
	print_hbm_lux_table(pSmart, "HEX");
#endif
	return;
}

static int smart_dimming_init(struct SMART_DIM *psmart)
{
	int lux_loop;

#ifdef SMART_DIMMING_DEBUG
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);
#endif
	id1 = (psmart->ldi_revision & 0x00FF0000) >> 16;
	id2 = (psmart->ldi_revision & 0x0000FF00) >> 8;
	id3 = psmart->ldi_revision & 0xFF;
	panel_rev = id3 & 0x0F;

	pr_err("%s : ++ panel_rev(%d)\n",__func__, panel_rev);

	mtp_sorting(psmart);
	set_max_lux_table(psmart->ldi_revision);

#ifdef SMART_DIMMING_DEBUG
	print_RGB_offset(psmart);
#endif

	psmart->vregout_voltage = get_vreg_voltage();

	/********************************************************************************************/
	/* 2.3.3 - each TP's gamma voltage calculation 											 	*/
	/* 1. VT, V255 : VREG1 - VREG1 * (OutputGamma + numerator_TP)/(denominator_TP) 			 	*/
	/* 2. VT1, VT7 : VREG1 - (VREG1 - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP) */
	/* 3. other VT : VT - (VT - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP)		*/
	/********************************************************************************************/
	TP_gamma_voltage_calc(psmart);

	/* 2.3.5 D - Gray Output Voltage */
	if (generate_gray_scale(psmart)) {
		pr_err(KERN_ERR "lcd smart dimming fail generate_gray_scale\n");
		return -EINVAL;
	}

	/* 3.7 - Generating lux_table */
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		/* To set brightness value */
		psmart->brightness_level = psmart->plux_table[lux_loop];
		/* To make lux table index*/
		psmart->gen_table[lux_loop].lux = psmart->plux_table[lux_loop];

		gamma_init(psmart,
			(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
			GAMMA_SET_MAX, lux_loop);
	}

	/* set 420CD max gamma table */
	memcpy(&(psmart->gen_table[LUMINANCE_MAX-1].gamma_setting),
			max_lux_table, GAMMA_SET_MAX);

	hbm_interpolation_init(psmart);

#ifdef SMART_DIMMING_DEBUG
	print_lux_table(psmart, "DEC");
	print_lux_table(psmart, "HEX");
#endif

	pr_err("%s done\n",__func__);

	return 0;
}

static void wrap_generate_hbm_gamma(struct smartdim_conf * conf, int hbm_brightness_level, char *cmd_str) {

	struct SMART_DIM *smart = conf->psmart;

	if (!smart) {
		pr_info("%s fail", __func__);
		return ;
	}

	/*auto_brightness is 13 to use 443cd of hbm step on color weakness mode*/
	if (hbm_brightness_level == HBM_MODE + 7)
		smart->hbm_brightness_level = -1;
	else
		smart->hbm_brightness_level = hbm_brightness_level - 6;
	generate_hbm_gamma(conf->psmart, cmd_str, GAMMA_SET_MAX);
}

/* ----------------------------------------------------------------------------
 * Wrapper functions for smart dimming
 * ----------------------------------------------------------------------------
 */
static void wrap_generate_gamma(struct smartdim_conf * conf, int cd, char *cmd_str) {

	struct SMART_DIM *smart = conf->psmart;

	if (!smart) {
		pr_err("%s fail", __func__);
		return ;
	}

	smart->brightness_level = cd;
	generate_gamma(conf->psmart, cmd_str, GAMMA_SET_MAX);
}

static void wrap_smart_dimming_init(struct smartdim_conf * conf) {

	struct SMART_DIM *smart = conf->psmart;

	if (!smart) {
		pr_err("%s fail", __func__);
		return ;
	}

	smart->plux_table = conf->lux_tab;
	smart->lux_table_max = conf->lux_tabsize;
	smart->ldi_revision = conf->man_id;
	smart->hbm_payload = conf->hbm_payload;

	if (smart->lux_table_max != LUMINANCE_MAX)
		pr_err("%s : [ERROR] LUMINANCE_MAX(%d) lux_table_max(%d) are different!\n",
				__func__, LUMINANCE_MAX, smart->lux_table_max);

	smart_dimming_init(smart);
}

struct smartdim_conf *smart_get_conf_S6E3FA5_AMS420MS01(void) {

	struct smartdim_conf * smartdim_conf;
	struct SMART_DIM *smart;

	smartdim_conf = kzalloc(sizeof(struct smartdim_conf), GFP_KERNEL);
	if (!smartdim_conf) {
		pr_err("%s allocation fail", __func__);
		goto out2;
	}

	smart = kzalloc(sizeof(struct SMART_DIM), GFP_KERNEL);
	if (!smart) {
		pr_err("%s allocation fail", __func__);
		goto out1;
	}

	smartdim_conf->psmart = smart;
	smartdim_conf->generate_gamma = wrap_generate_gamma;
	smartdim_conf->generate_hbm_gamma = wrap_generate_hbm_gamma;
	smartdim_conf->init = wrap_smart_dimming_init;
	smartdim_conf->get_min_lux_table = NULL;
	smartdim_conf->mtp_buffer = smart->MTP_ORIGN;
	smartdim_conf->print_aid_log = print_aid_log;

	return smartdim_conf;

out1:
	kfree(smartdim_conf);
out2:
	return NULL;
}

/* ----------------------------------------------------------------------------
 * END - Wrapper
 * ----------------------------------------------------------------------------
 */

/* ============================================================================
 *  HMT
 * ============================================================================
 */

static void print_lux_table_hmt(struct SMART_DIM *psmart)
{
	int lux_loop;
	int cnt;
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);

	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
			snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %3d",
				psmart->hmt_gen_table[lux_loop].gamma_setting[cnt]);

		pr_err("lux : %3d  %s\n", psmart->plux_table[lux_loop], pBuffer);
		memset(pBuffer, 0x00, 256);
	}
}

static void print_aid_log_hmt(struct smartdim_conf *conf)
{
	pr_err("== print_aid_log_hmt ==\n");
	print_RGB_offset(conf->psmart);
	print_lux_table_hmt(conf->psmart);
	pr_err("\n");
}

static int get_base_luminance_hmt(int brightness_level) {

	int cnt;
	int base_luminance[HMT_LUMINANCE_MAX][2];

	if (panel_rev == 0x01)
		memcpy(base_luminance, base_luminance_reverse_hmt_single, sizeof(base_luminance_reverse_hmt_single));
	else
		memcpy(base_luminance, base_luminance_reverse_hmt_single, sizeof(base_luminance_reverse_hmt_single));

	for (cnt = 0; cnt < HMT_LUMINANCE_MAX; cnt++)
		if (base_luminance[cnt][0] == brightness_level)
			return base_luminance[cnt][1];

	return -1;
}

static int get_gradation_offset_hmt(int table_index, int index)
{
	if (panel_rev == 0x01)
		return gradation_offset_reverse_hmt_single[table_index][index];
	else
		return  gradation_offset_reverse_hmt_single[table_index][index];
}

static int get_rgb_offset_hmt(int table_index, int index)
{
	if (panel_rev == 0x01)
		return rgb_offset_reverse_hmt_single[table_index][index];
	else
		return rgb_offset_reverse_hmt_single[table_index][index];
}

static void TP_L_calc_hmt(struct SMART_DIM *pSmart, long long *L, int base_level)
{
	long long temp_cal_data = 0;
	int point_index;
	int cnt;

	for (cnt = 0; cnt < V_MAX; cnt++) {
		point_index = INFLECTION_VOLTAGE_ARRAY[cnt];
		temp_cal_data =
			((long long)(candela_coeff_2p15[point_index])) *
			((long long)(base_level));

		L[cnt] = temp_cal_data;
	}
}

static void TP_M_GRAY_calc_hmt(struct SMART_DIM *pSmart, long long *L, int *M_GRAY, int table_index)
{
	int i;

	M_GRAY[VT] = 0;
	M_GRAY[V1] = 1;

	for (i = V7; i < V_MAX; i++) {
		/* 3.6 - TP's Voltage Search */
		if (searching_function(L[i],
			&(M_GRAY[i]), get_gamma_curve(pSmart))) {
			pr_err("%s searching functioin error cnt:%d\n",
			__func__, i);
		}
	}

	/* 2.2.6 - add offset for Candela Gamma Compensation (V255~V7) */
	for (i = V255; i >= V7; i--) {
		if (table_index == -1) {
			table_index = HMT_LUMINANCE_MAX-1;
			pr_err("%s : fail candela table_index cnt : %d brightness %d\n",
				__func__, i, pSmart->brightness_level);
		}
		M_GRAY[i] += get_gradation_offset_hmt(table_index, V255-i);
	}
}

static void Color_shift_compensation_hmt(struct SMART_DIM *pSmart, int *gamma_setting, int table_index)
{
	int i;
	int level_255_temp_MSB = 0;
	int level_V255 = 0;

	for (i = 0; i < RGB_COMPENSATION; i++) {
		if (table_index == -1) {
			table_index = HMT_LUMINANCE_MAX-1;
			pr_err("%s : fail RGB table_index cnt : %d brightness %d\n",
				__func__, i, pSmart->brightness_level);
		}

		if (i < 3) {
			level_V255 = gamma_setting[i * 2] << 8 | gamma_setting[(i * 2) + 1];
			level_V255 += get_rgb_offset_hmt(table_index, i);
			level_255_temp_MSB = level_V255 / 256;

			gamma_setting[i * 2] = level_255_temp_MSB & 0xff;
			gamma_setting[(i * 2) + 1] = level_V255 & 0xff;
		} else {
			gamma_setting[i+3] += get_rgb_offset_hmt(table_index, i);
		}
	}
}

#define TABLE_MAX  (V_MAX-1)

static void gamma_init_hmt(struct SMART_DIM *pSmart, char *str, int size, int table_index)
{
#ifdef SMART_DIMMING_DEBUG
	int i;
#endif
	long long L[V_MAX] = {0, };
	int M_GRAY[V_MAX] = {0, };
	int gamma_setting[GAMMA_SET_MAX];
	int base_level = 0;
	int cnt;

	pr_debug("%s : start !! table_index(%d)\n",__func__,table_index);

	/* get base luminance */
	base_level = get_base_luminance_hmt(pSmart->brightness_level);
	if (base_level < 0)
		pr_err("%s : can not find base luminance!!\n", __func__);

	/* 2.2.5 (F) TP's Luminance  (x4194304) */
	TP_L_calc_hmt(pSmart, L, base_level);

	/* 3.5.1 M-Gray */
	TP_M_GRAY_calc_hmt(pSmart, L, M_GRAY, table_index);

#ifdef SMART_DIMMING_DEBUG
	pr_err("\n brightness_level (%d) \n %16s %8s\n",
			   pSmart->brightness_level, "L", "M_GRAY");

	for (i=0; i<V_MAX; i++)
		pr_err("%5s %11llu %8d\n", V_LIST[i], L[i], M_GRAY[i]);
#endif

	/* 3.7.1 - Generate Gamma code */
	TP_gamma_code_calc(pSmart, M_GRAY, gamma_setting);

	/* 3.7.2 - Color Shift (RGB compensation) */
	Color_shift_compensation_hmt(pSmart, gamma_setting, table_index);

	/* subtration MTP_OFFSET value from generated gamma table */
	mtp_offset_substraction(pSmart, gamma_setting);

	/* To avoid overflow */
	for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
		str[cnt] = gamma_setting[cnt];
}

static void generate_gamma_hmt(struct SMART_DIM *psmart, char *str, int size)
{
	int lux_loop;
	struct illuminance_table *ptable = (struct illuminance_table *)
						(&(psmart->hmt_gen_table));

	/* searching already generated gamma table */
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		if (ptable[lux_loop].lux == psmart->brightness_level) {
			memcpy(str, &(ptable[lux_loop].gamma_setting), size);
			break;
		}
	}

	/* searching fail... Setting 150CD value on gamma table */
	if (lux_loop == psmart->lux_table_max) {
		pr_err("%s searching fail lux : %d\n", __func__,
				psmart->brightness_level);
		memcpy(str, max_lux_table, size);
	}

#ifdef SMART_DIMMING_DEBUG
	if (lux_loop != psmart->lux_table_max)
		pr_err("%s searching ok index : %d lux : %d", __func__,
			lux_loop, ptable[lux_loop].lux);
#endif
}

static int smart_dimming_init_hmt(struct SMART_DIM *psmart)
{
	int lux_loop;

#ifdef SMART_DIMMING_DEBUG
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);
#endif
	id1 = (psmart->ldi_revision & 0x00FF0000) >> 16;
	id2 = (psmart->ldi_revision & 0x0000FF00) >> 8;
	id3 = psmart->ldi_revision & 0xFF;
	panel_rev = id3 & 0x0F;

	pr_err("%s : ++ panel_rev(%d)\n",__func__, panel_rev);

	mtp_sorting(psmart);
	set_max_lux_table(psmart->ldi_revision);

#ifdef SMART_DIMMING_DEBUG
	print_RGB_offset(psmart);
#endif

	psmart->vregout_voltage = get_vreg_voltage();

	/********************************************************************************************/
	/* 2.3.3 - each TP's gamma voltage calculation 											 	*/
	/* 1. VT, V255 : VREG1 - VREG1 * (OutputGamma + numerator_TP)/(denominator_TP) 			 	*/
	/* 2. VT1, VT7 : VREG1 - (VREG1 - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP) */
	/* 3. other VT : VT - (VT - V_nextTP) * (OutputGamma + numerator_TP)/(denominator_TP)		*/
	/********************************************************************************************/
	TP_gamma_voltage_calc(psmart);

	/* 2.3.5 D - Gray Output Voltage */
	if (generate_gray_scale(psmart)) {
		pr_err(KERN_ERR "lcd smart dimming fail generate_gray_scale\n");
		return -EINVAL;
	}

	/* 3.7 - Generating lux_table */
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		/* To set brightness value */
		psmart->brightness_level = psmart->plux_table[lux_loop];
		/* To make lux table index*/
		psmart->hmt_gen_table[lux_loop].lux = psmart->plux_table[lux_loop];

		gamma_init_hmt(psmart,
			(char *)(&(psmart->hmt_gen_table[lux_loop].gamma_setting)),
			GAMMA_SET_MAX, lux_loop);
	}

#ifdef SMART_DIMMING_DEBUG
	print_lux_table_hmt(psmart);
#endif

	pr_err("%s done\n",__func__);

	return 0;
}

static void wrap_generate_gamma_hmt(struct smartdim_conf * conf, int cd, char *cmd_str) {

	struct SMART_DIM *smart = conf->psmart;

	if (!smart) {
		pr_err("%s fail", __func__);
		return ;
	}

	smart->brightness_level = cd;
	generate_gamma_hmt(conf->psmart, cmd_str, GAMMA_SET_MAX);
}

static void wrap_smart_dimming_init_hmt(struct smartdim_conf * conf) {

	struct SMART_DIM *smart = conf->psmart;

	if (!smart) {
		pr_err("%s fail", __func__);
		return;
	}

	smart->plux_table = conf->lux_tab;
	smart->lux_table_max = conf->lux_tabsize;
	smart->ldi_revision = conf->man_id;
	smart_dimming_init_hmt(smart);
}

struct smartdim_conf *smart_get_conf_S6E3FA5_AMS420MS01_hmt(void) {

	struct smartdim_conf *smartdim_conf_hmt;
	struct SMART_DIM *smart_hmt;

	smartdim_conf_hmt = kzalloc(sizeof(struct smartdim_conf), GFP_KERNEL);
	if (!smartdim_conf_hmt) {
		pr_err("%s allocation fail", __func__);
		goto out2;
	}

	smart_hmt = kzalloc(sizeof(struct SMART_DIM), GFP_KERNEL);
	if (!smart_hmt) {
		pr_err("%s allocation fail", __func__);
		goto out1;
	}

	smartdim_conf_hmt->psmart = smart_hmt;
	smartdim_conf_hmt->generate_gamma = wrap_generate_gamma_hmt;
	smartdim_conf_hmt->init = wrap_smart_dimming_init_hmt;
	smartdim_conf_hmt->get_min_lux_table = NULL;
	smartdim_conf_hmt->mtp_buffer = (char *)(&smart_hmt->MTP_ORIGN);
	smartdim_conf_hmt->print_aid_log = print_aid_log_hmt;

	return smartdim_conf_hmt;

out1:
	kfree(smartdim_conf_hmt);
out2:
	return NULL;
}
