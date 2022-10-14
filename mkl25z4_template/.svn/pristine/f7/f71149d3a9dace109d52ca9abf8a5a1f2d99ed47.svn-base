/*
 * Name: cube_master.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "bme.h"
#include "acc.h"
#include "led.h"

#include "cube_lut.h"

#define DAC0_DAT0			(*((volatile uint16_t *) 0x4003F000ul))

#define ABS(a)				(((a) > 0) ? (a) : -(a))
#define SWAP(a, b)			do { (a) ^= (b); (b) ^= (a); (a) ^= (b); } while (0)

#define POT_MEAS_PERIOD		3750u
#define POT_MEAS_OFFSET_X	0u
#define POT_MEAS_OFFSET_Y	1875u

#define POT_ANG_COEFF		1.408f

#define CUBE_DIM			80.0f

#define ANG_AUTO_STEP_X		1
#define ANG_AUTO_STEP_Y		1
#define ANG_AUTO_STEP_Z		1

#define CANVAS_OFFSET		2048

#define CUBE_ORIG_INIT		{ \
								.f0.x = -CUBE_DIM,	.f0.y = -CUBE_DIM,	.f0.z = CUBE_DIM,	\
								.f1.x = CUBE_DIM,	.f1.y = -CUBE_DIM,	.f1.z = CUBE_DIM,	\
								.f2.x = -CUBE_DIM,	.f2.y = CUBE_DIM,	.f2.z = CUBE_DIM,	\
								.f3.x = CUBE_DIM,	.f3.y = CUBE_DIM,	.f3.z = CUBE_DIM,	\
								.b0.x = -CUBE_DIM,	.b0.y = -CUBE_DIM,	.b0.z = -CUBE_DIM,	\
								.b1.x = CUBE_DIM,	.b1.y = -CUBE_DIM,	.b1.z = -CUBE_DIM,	\
								.b2.x = -CUBE_DIM,	.b2.y = CUBE_DIM,	.b2.z = -CUBE_DIM,	\
								.b3.x = CUBE_DIM,	.b3.y = CUBE_DIM,	.b3.z = -CUBE_DIM,	\
							}

#define ANG_INIT			{.x = 0, .y = 0, .z = 0}

#define ACC_RST_DELAY		48000ul

#define ACC_RANGE			ACC_XYZDATCFG_2G
#define ACC_SAMPLE_RATE		ACC_CR1_050_00HZ

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} ANG;

typedef struct {
	float x;
	float y;
	float z;
} VERTEX_3D;

typedef struct {
	VERTEX_3D f0;
	VERTEX_3D f1;
	VERTEX_3D f2;
	VERTEX_3D f3;
	VERTEX_3D b0;
	VERTEX_3D b1;
	VERTEX_3D b2;
	VERTEX_3D b3;
} CUBE_3D;

typedef struct {
	int16_t x;
	int16_t y;
} VERTEX_2D;

typedef struct {
	VERTEX_2D f0;
	VERTEX_2D f1;
	VERTEX_2D f2;
	VERTEX_2D f3;
	VERTEX_2D b0;
	VERTEX_2D b1;
	VERTEX_2D b2;
	VERTEX_2D b3;
} CUBE_2D;

typedef enum {
	APP_MODE_AUTO,
	APP_MODE_STOP,
	APP_MODE_POT,
	APP_MODE_ACC
} APP_MODE;

volatile APP_MODE app_mode = APP_MODE_AUTO;

volatile ACC_SPI_BULK_SAMP_BLK_RX acc_samp_blk_rx;
volatile ACC_SPI_BULK_SAMP_BLK_TX acc_samp_blk_tx;

const float lut_sin[CUBE_LUT_TRIG_LEN] = {CUBE_LUT_SIN};
const float lut_cos[CUBE_LUT_TRIG_LEN] = {CUBE_LUT_COS};

void __attribute__ ((interrupt)) PORTA_IRQHandler(void);
void __attribute__ ((interrupt)) PORTD_IRQHandler(void);
void __attribute__ ((interrupt)) DMA0_IRQHandler(void);

void line_draw_direct(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void line_draw_inverse(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void line_draw(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

void vertex_rotate_x(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang);
void vertex_rotate_y(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang);
void vertex_rotate_z(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang);
void vertex_rotate(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, const ANG *ang);

void cube_rotate(const CUBE_3D *cube_in, CUBE_3D *cube_out, const ANG *ang);
void cube_proj_ortho(const CUBE_3D *cube_in, CUBE_2D *cube_out);
void cube_draw(const CUBE_2D *cube);

void ang_get_auto(ANG *ang);
void ang_get_pot(ANG *ang);
void ang_get_acc(ANG *ang);

uint8_t acc_reg_rd(uint8_t addr);
void acc_reg_wr(uint8_t addr, uint8_t val);
void acc_samp_blk_rd_init(void);
void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1);
uint8_t acc_spi_trailer_rdwr(uint8_t data_out);
void acc_enable(void);

void __attribute__ ((interrupt)) PORTA_IRQHandler(void)
{
	PORTA->ISFR = 1ul << 4;

	switch (app_mode) {
	case APP_MODE_AUTO:
		app_mode = APP_MODE_STOP;
		led_bindisp(2);
		break;
	case APP_MODE_STOP:
		app_mode = APP_MODE_POT;
		led_bindisp(4);
		break;
	case APP_MODE_POT:
		app_mode = APP_MODE_ACC;
		led_bindisp(8);
		break;
	case APP_MODE_ACC:
		app_mode = APP_MODE_AUTO;
		led_bindisp(1);
		break;
	default:
		break;
	}
}

void __attribute__ ((interrupt)) PORTD_IRQHandler(void)
{
	PORTD->ISFR = 1ul << 6;

	acc_samp_blk_rd_init();
}

void __attribute__ ((interrupt)) DMA0_IRQHandler(void)
{
	DMA0->DMA[0].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[1].DSR_BCR = DMA_DSR_BCR_DONE_MASK;

	FGPIOE->PSOR = 1ul << 4;
}

void line_draw_direct(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int16_t dx, dy, iter, iter_lim, res, arg, arg_step;
	float grad, icept;

	dx = x2 - x1;
	dy = y2 - y1;

	grad = (float) dy / dx;
	icept = y1 - grad * x1;
	iter_lim = ABS(dx);
	arg = x1;
	arg_step = (x2 > x1) ? 1 : -1;

	for (iter = 0; iter <= iter_lim; iter++) {
		res = grad * arg + icept;
		arg += arg_step;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPTEF_SHIFT, 1));
		SPI0->D = (res >> 6) | 0x80;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPRF_SHIFT, 1));
		SPI0->D;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPTEF_SHIFT, 1));
		SPI0->D = res & 0x3F;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPRF_SHIFT, 1));
		SPI0->D;

		DAC0_DAT0 = arg;
	}
}

void line_draw_inverse(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int16_t dx, dy, iter, iter_lim, res, arg, arg_step;
	float grad, icept;

	dx = x2 - x1;
	dy = y2 - y1;

	grad = (float) dx / dy;
	icept = x1 - grad * y1;
	iter_lim = ABS(dy);
	arg = y1;
	arg_step = (y2 > y1) ? 1 : -1;

	for (iter = 0; iter <= iter_lim; iter++) {
		res = grad * arg + icept;
		arg += arg_step;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPTEF_SHIFT, 1));
		SPI0->D = (arg >> 6) | 0x80;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPRF_SHIFT, 1));
		SPI0->D;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPTEF_SHIFT, 1));
		SPI0->D = arg & 0x3F;

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPRF_SHIFT, 1));
		SPI0->D;

		DAC0_DAT0 = res;
	}
}

void line_draw(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int16_t dx, dy;

	dx = x2 - x1;
	dy = y2 - y1;

	if (ABS(dx) > ABS(dy)) {
		line_draw_direct(x1, y1, x2, y2);
	} else {
		line_draw_inverse(x1, y1, x2, y2);
	}
}

void vertex_rotate_x(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang)
{
	float vert_in_y = vert_in->y;

	vert_out->x = vert_in->x;
	vert_out->y = vert_in_y * lut_cos[ang] - vert_in->z * lut_sin[ang];
	vert_out->z = vert_in_y * lut_sin[ang] + vert_in->z * lut_cos[ang];
}

void vertex_rotate_y(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang)
{
	float vert_in_x = vert_in->x;

	vert_out->x = vert_in->z * lut_sin[ang] + vert_in_x * lut_cos[ang];
	vert_out->y = vert_in->y;
	vert_out->z = vert_in->z * lut_cos[ang] - vert_in_x * lut_sin[ang];
}

void vertex_rotate_z(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, int16_t ang)
{
	float vert_in_x = vert_in->x;

	vert_out->x = vert_in_x * lut_cos[ang] - vert_in->y * lut_sin[ang];
	vert_out->y = vert_in_x * lut_sin[ang] + vert_in->y * lut_cos[ang];
	vert_out->z = vert_in->z;
}

void vertex_rotate(const VERTEX_3D *vert_in, VERTEX_3D *vert_out, const ANG *ang)
{
	vertex_rotate_x(vert_in, vert_out, ang->x);
	vertex_rotate_y(vert_out, vert_out, ang->y);
	vertex_rotate_z(vert_out, vert_out, ang->z);
}

void cube_rotate(const CUBE_3D *cube_in, CUBE_3D *cube_out, const ANG *ang)
{
	vertex_rotate(&cube_in->f0, &cube_out->f0, ang);
	vertex_rotate(&cube_in->f1, &cube_out->f1, ang);
	vertex_rotate(&cube_in->f2, &cube_out->f2, ang);
	vertex_rotate(&cube_in->f3, &cube_out->f3, ang);

	vertex_rotate(&cube_in->b0, &cube_out->b0, ang);
	vertex_rotate(&cube_in->b1, &cube_out->b1, ang);
	vertex_rotate(&cube_in->b2, &cube_out->b2, ang);
	vertex_rotate(&cube_in->b3, &cube_out->b3, ang);
}

void cube_proj_ortho(const CUBE_3D *cube_in, CUBE_2D *cube_out)
{
	cube_out->f0.x = (int16_t) cube_in->f0.x + CANVAS_OFFSET;
	cube_out->f0.y = (int16_t) cube_in->f0.y + CANVAS_OFFSET;

	cube_out->f1.x = (int16_t) cube_in->f1.x + CANVAS_OFFSET;
	cube_out->f1.y = (int16_t) cube_in->f1.y + CANVAS_OFFSET;

	cube_out->f2.x = (int16_t) cube_in->f2.x + CANVAS_OFFSET;
	cube_out->f2.y = (int16_t) cube_in->f2.y + CANVAS_OFFSET;

	cube_out->f3.x = (int16_t) cube_in->f3.x + CANVAS_OFFSET;
	cube_out->f3.y = (int16_t) cube_in->f3.y + CANVAS_OFFSET;

	cube_out->b0.x = (int16_t) cube_in->b0.x + CANVAS_OFFSET;
	cube_out->b0.y = (int16_t) cube_in->b0.y + CANVAS_OFFSET;

	cube_out->b1.x = (int16_t) cube_in->b1.x + CANVAS_OFFSET;
	cube_out->b1.y = (int16_t) cube_in->b1.y + CANVAS_OFFSET;

	cube_out->b2.x = (int16_t) cube_in->b2.x + CANVAS_OFFSET;
	cube_out->b2.y = (int16_t) cube_in->b2.y + CANVAS_OFFSET;

	cube_out->b3.x = (int16_t) cube_in->b3.x + CANVAS_OFFSET;
	cube_out->b3.y = (int16_t) cube_in->b3.y + CANVAS_OFFSET;
}

void cube_draw(const CUBE_2D *cube)
{
	line_draw(cube->f0.x, cube->f0.y, cube->f1.x, cube->f1.y);
	line_draw(cube->f1.x, cube->f1.y, cube->f3.x, cube->f3.y);
	line_draw(cube->f3.x, cube->f3.y, cube->f2.x, cube->f2.y);
	line_draw(cube->f2.x, cube->f2.y, cube->f0.x, cube->f0.y);

	line_draw(cube->b0.x, cube->b0.y, cube->b1.x, cube->b1.y);
	line_draw(cube->b1.x, cube->b1.y, cube->b3.x, cube->b3.y);
	line_draw(cube->b3.x, cube->b3.y, cube->b2.x, cube->b2.y);
	line_draw(cube->b2.x, cube->b2.y, cube->b0.x, cube->b0.y);

	line_draw(cube->f0.x, cube->f0.y, cube->b0.x, cube->b0.y);
	line_draw(cube->f1.x, cube->f1.y, cube->b1.x, cube->b1.y);
	line_draw(cube->f2.x, cube->f2.y, cube->b2.x, cube->b2.y);
	line_draw(cube->f3.x, cube->f3.y, cube->b3.x, cube->b3.y);
}

void ang_get_auto(ANG *ang)
{
	ang->x += ANG_AUTO_STEP_X;
	ang->x %= CUBE_LUT_TRIG_LEN;
	ang->x = (ang->x < 0) ? CUBE_LUT_TRIG_LEN + ang->x : ang->x;

	ang->y += ANG_AUTO_STEP_Y;
	ang->y %= CUBE_LUT_TRIG_LEN;
	ang->y = (ang->y < 0) ? CUBE_LUT_TRIG_LEN + ang->y : ang->y;

	ang->z += ANG_AUTO_STEP_Z;
	ang->z %= CUBE_LUT_TRIG_LEN;
	ang->z = (ang->z < 0) ? CUBE_LUT_TRIG_LEN + ang->z : ang->z;
}

void ang_get_pot(ANG *ang)
{
	ang->x = ADC0->R[0] * POT_ANG_COEFF;
	ang->y = ADC0->R[1] * POT_ANG_COEFF;
	ang->z = 0;
}

void ang_get_acc(ANG *ang)
{
	int16_t samp;

	samp = acc_samp_blk_rx.samp_blk.axis_x_hi;
	samp <<= 8;
	samp |= acc_samp_blk_rx.samp_blk.axis_x_lo;

	ang->y = 0.00547f * samp + 179;

	samp = acc_samp_blk_rx.samp_blk.axis_y_hi;
	samp <<= 8;
	samp |= acc_samp_blk_rx.samp_blk.axis_y_lo;

	ang->x = 0.00547f * samp + 179;

	ang->z = 0;
}

uint8_t acc_reg_rd(uint8_t addr)
{
	uint8_t data_in;

	FGPIOE->PCOR = 1ul << 4;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_RD(addr), ACC_SPI_MAKE_CTRL1(addr));
	data_in = acc_spi_trailer_rdwr(0);

	FGPIOE->PSOR = 1ul << 4;

	return data_in;
}

void acc_reg_wr(uint8_t addr, uint8_t val)
{
	FGPIOE->PCOR = 1ul << 4;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_WR(addr), ACC_SPI_MAKE_CTRL1(addr));
	acc_spi_trailer_rdwr(val);

	FGPIOE->PSOR = 1ul << 4;
}

void acc_samp_blk_rd_init(void)
{
	FGPIOE->PCOR = 1ul << 4;

	DMA0->DMA[0].DAR = (uint32_t) &acc_samp_blk_rx;
	DMA0->DMA[0].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(acc_samp_blk_rx));
	DMA0->DMA[0].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SSIZE(1) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;

	DMA0->DMA[1].SAR = (uint32_t) &acc_samp_blk_tx;
	DMA0->DMA[1].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(acc_samp_blk_tx));
	DMA0->DMA[1].DCR = DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;
}

void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1)
{
	SPI1->S;
	SPI1->D = ctrl0;

	while (!BME_UBFX_B(&SPI1->S, SPI_S_SPRF_SHIFT, 1));
	SPI1->D;

	SPI1->S;
	SPI1->D = ctrl1;

	while (!BME_UBFX_B(&SPI1->S, SPI_S_SPRF_SHIFT, 1));
	SPI1->D;
}

uint8_t acc_spi_trailer_rdwr(uint8_t data_out)
{
	SPI1->S;
	SPI1->D = data_out;

	while (!BME_UBFX_B(&SPI1->S, SPI_S_SPRF_SHIFT, 1));
	return SPI1->D;
}

void acc_enable(void)
{
	PORTE->PCR[5] = PORT_PCR_MUX(1);
	FGPIOE->PDDR |= 1ul << 5;
	FGPIOE->PSOR = 1ul << 5;

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0].LDVAL = ACC_RST_DELAY - 1;
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;

	while (!(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK));
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;

	FGPIOE->PCOR = 1ul << 5;

	while (!(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK));

	PORTD->PCR[7] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;
	PORTE->PCR[3] = PORT_PCR_MUX(5);
	PORTE->PCR[2] = PORT_PCR_MUX(2);

	PORTE->PCR[4] = PORT_PCR_MUX(1);
	FGPIOE->PDDR |= 1ul << 4;
	FGPIOE->PSOR = 1ul << 4;

	PORTD->PCR[6] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(10);

	SPI1->BR = SPI_BR_SPPR(5) | SPI_BR_SPR(2);
	SPI1->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;

	acc_reg_wr(ACC_ADDR_XYZDATCFG, ACC_RANGE);
	acc_reg_wr(ACC_ADDR_CR4, ACC_CR4_INT_EN_DRDY);
	acc_reg_wr(ACC_ADDR_CR5, ACC_CR5_INT_DRDY_INT1);
	acc_reg_wr(ACC_ADDR_CR1, ACC_SAMPLE_RATE);

	SPI1->C1 = 0;
	SPI1->C2 = SPI_C2_TXDMAE_MASK | SPI_C2_RXDMAE_MASK;
	SPI1->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;

	acc_samp_blk_tx.ctrl0 = ACC_SPI_MAKE_CTRL0_RD(ACC_ADDR_SAMP_BLK);
	acc_samp_blk_tx.ctrl1 = ACC_SPI_MAKE_CTRL1(ACC_ADDR_SAMP_BLK);

	NVIC_SetPriority(DMA0_IRQn, 0);
	NVIC_EnableIRQ(DMA0_IRQn);

	DMA0->DMA[0].SAR = (uint32_t) &SPI1->D;
	DMA0->DMA[1].DAR = (uint32_t) &SPI1->D;

	DMAMUX0->CHCFG[0] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(18);
	DMAMUX0->CHCFG[1] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(19);

	NVIC_SetPriority(PORTD_IRQn, 0);
	NVIC_EnableIRQ(PORTD_IRQn);
}

int main(void)
{
	CUBE_3D cube_adj, cube_orig = CUBE_ORIG_INIT;
	CUBE_2D cube_proj;

	ANG ang = ANG_INIT;

	wdog_init(WDOG_CONF_DIS);

	led_init();
	led_bindisp(1);

	acc_enable();

	NVIC_SetPriority(PORTA_IRQn, 1);
	NVIC_EnableIRQ(PORTA_IRQn);

	PORTA->PCR[4] = PORT_PCR_IRQC(10) | PORT_PCR_MUX(1);

	ADC0->CFG1 = ADC_CFG1_ADICLK(3);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC1[0] = 11;
	ADC0->SC1[1] = 12;

	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[0].CnV = POT_MEAS_OFFSET_X;
	TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[1].CnV = POT_MEAS_OFFSET_Y;
	TPM1->MOD = POT_MEAS_PERIOD - 1;
	TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);

	DAC0->C0 = DAC_C0_DACEN_MASK;

	SPI0->BR = SPI_BR_SPPR(1);
	SPI0->C2 = SPI_C2_MODFEN_MASK;
	SPI0->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;

	PORTD->PCR[0] = PORT_PCR_MUX(2);
	PORTD->PCR[1] = PORT_PCR_MUX(2);
	PORTD->PCR[2] = PORT_PCR_MUX(2);
	PORTD->PCR[3] = PORT_PCR_MUX(2);

	while (1) {
		cube_rotate(&cube_orig, &cube_adj, &ang);
		cube_proj_ortho(&cube_adj, &cube_proj);
		cube_draw(&cube_proj);

		switch (app_mode) {
		case APP_MODE_AUTO:
			ang_get_auto(&ang);
			break;
		case APP_MODE_STOP:
			break;
		case APP_MODE_POT:
			ang_get_pot(&ang);
			break;
		case APP_MODE_ACC:
			ang_get_acc(&ang);
			break;
		default:
			break;
		}
	}

	return 0;
}
