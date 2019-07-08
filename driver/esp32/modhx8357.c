//////////////////////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////////////////////

#include "../include/common.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lvgl/src/lv_hal/lv_hal_disp.h"

//////////////////////////////////////////////////////////////////////////////
// ILI9341 requires specific lv_conf resolution and color depth
//////////////////////////////////////////////////////////////////////////////

#if LV_COLOR_DEPTH != 16
#error "modILI9341: LV_COLOR_DEPTH must be set to 16!"
#endif

//////////////////////////////////////////////////////////////////////////////
// ILI9341 Module definitions
//////////////////////////////////////////////////////////////////////////////
#define HX8357_NOP                0x00  ///< No op
#define HX8357_SWRESET            0x01  ///< software reset
#define HX8357_RDDID              0x04  ///< Read ID
#define HX8357_RDDST              0x09  ///< (unknown)

#define HX8357_RDPOWMODE          0x0A  ///< Read power mode Read power mode
#define HX8357_RDMADCTL           0x0B  ///< Read MADCTL
#define HX8357_RDCOLMOD           0x0C  ///< Column entry mode
#define HX8357_RDDIM              0x0D  ///< Read display image mode
#define HX8357_RDDSDR             0x0F  ///< Read dosplay signal mode

#define HX8357_SLPIN              0x10  ///< Enter sleep mode
#define HX8357_SLPOUT             0x11  ///< Exit sleep mode
#define HX8357B_PTLON             0x12  ///< Partial mode on
#define HX8357B_NORON             0x13  ///< Normal mode

#define HX8357_INVOFF             0x20  ///< Turn off invert
#define HX8357_INVON              0x21  ///< Turn on invert
#define HX8357_DISPOFF            0x28  ///< Display on
#define HX8357_DISPON             0x29  ///< Display off

#define HX8357_CASET              0x2A  ///< Column addr set
#define HX8357_PASET              0x2B  ///< Page addr set
#define HX8357_RAMWR              0x2C  ///< Write VRAM
#define HX8357_RAMRD              0x2E  ///< Read VRAm

#define HX8357B_PTLAR             0x30  ///< (unknown)
#define HX8357_TEON               0x35  ///< Tear enable on
#define HX8357_TEARLINE           0x44  ///< (unknown)
#define HX8357_MADCTL             0x36  ///< Memory access control
#define HX8357_COLMOD             0x3A  ///< Color mode

#define HX8357_SETOSC             0xB0  ///< Set oscillator
#define HX8357_SETPWR1            0xB1  ///< Set power control
#define HX8357B_SETDISPLAY        0xB2  ///< Set display mode
#define HX8357_SETRGB             0xB3  ///< Set RGB interface
#define HX8357D_SETCOM            0xB6  ///< Set VCOM voltage

#define HX8357B_SETDISPMODE       0xB4  ///< Set display mode
#define HX8357D_SETCYC            0xB4  ///< Set display cycle reg
#define HX8357B_SETOTP            0xB7  ///< Set OTP memory
#define HX8357D_SETC              0xB9  ///< Enable extension command

#define HX8357B_SET_PANEL_DRIVING 0xC0  ///< Set panel drive mode
#define HX8357D_SETSTBA           0xC0  ///< Set source option
#define HX8357B_SETDGC            0xC1  ///< Set DGC settings
#define HX8357B_SETID             0xC3  ///< Set ID
#define HX8357B_SETDDB            0xC4  ///< Set DDB
#define HX8357B_SETDISPLAYFRAME   0xC5  ///< Set display frame
#define HX8357B_GAMMASET          0xC8  ///< Set Gamma correction
#define HX8357B_SETCABC           0xC9  ///< Set CABC
#define HX8357_SETPANEL           0xCC  ///< Set Panel

#define HX8357B_SETPOWER          0xD0  ///< Set power control
#define HX8357B_SETVCOM           0xD1  ///< Set VCOM
#define HX8357B_SETPWRNORMAL      0xD2  ///< Set power normal

#define HX8357B_RDID1             0xDA  ///< Read ID #1
#define HX8357B_RDID2             0xDB  ///< Read ID #2
#define HX8357B_RDID3             0xDC  ///< Read ID #3
#define HX8357B_RDID4             0xDD  ///< Read ID #4

#define HX8357D_SETGAMMA          0xE0  ///< Set Gamma

#define HX8357B_SETGAMMA          0xC8 ///< Set Gamma
#define HX8357B_SETPANELRELATED   0xE9 ///< Set panel related

// Plan is to move this to GFX header (with different prefix), though
// defines will be kept here for existing code that might be referencing
// them. Some additional ones are in the ILI9341 lib -- add all in GFX!
// Color definitions
#define	HX8357_BLACK   0x0000 ///< BLACK color for drawing graphics
#define	HX8357_BLUE    0x001F ///< BLUE color for drawing graphics
#define	HX8357_RED     0xF800 ///< RED color for drawing graphics
#define	HX8357_GREEN   0x07E0 ///< GREEN color for drawing graphics
#define HX8357_CYAN    0x07FF ///< CYAN color for drawing graphics
#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
#define HX8357_YELLOW  0xFFE0 ///< YELLOW color for drawing graphics
#define HX8357_WHITE   0xFFFF ///< WHITE color for drawing graphics

typedef struct {
    mp_obj_base_t base;
    spi_device_handle_t spi;
    uint8_t mhz;
    uint8_t spihost;
    uint8_t miso;
    uint8_t mosi;
    uint8_t clk;
    uint8_t cs;
    uint8_t dc;
    uint8_t rst;
    uint8_t backlight;

} ILI9341_t;

// Unfortunately, lvgl doesnt pass user_data to callbacks, so we use this global.
// This means we can have only one active display driver instance, pointed by this global.
STATIC ILI9341_t *g_ILI9341 = NULL;

STATIC mp_obj_t ILI9341_make_new(const mp_obj_type_t *type,
                                 size_t n_args,
                                 size_t n_kw,
                                 const mp_obj_t *all_args);

STATIC mp_obj_t mp_init_ILI9341(mp_obj_t self_in);

STATIC mp_obj_t mp_activate_ILI9341(mp_obj_t self_in)
{
    ILI9341_t *self = MP_OBJ_TO_PTR(self_in);
    g_ILI9341 = self;
    return mp_const_none;
}

STATIC void ili9431_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_init_ILI9341_obj, mp_init_ILI9341);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_activate_ILI9341_obj, mp_activate_ILI9341);
DEFINE_PTR_OBJ(ili9431_flush);

STATIC const mp_rom_map_elem_t ILI9341_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_init_ILI9341_obj) },
    { MP_ROM_QSTR(MP_QSTR_activate), MP_ROM_PTR(&mp_activate_ILI9341_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&PTR_OBJ(ili9431_flush)) },
};

STATIC MP_DEFINE_CONST_DICT(ILI9341_locals_dict, ILI9341_locals_dict_table);

STATIC const mp_obj_type_t ILI9341_type = {
    { &mp_type_type },
    .name = MP_QSTR_ILI9341,
    //.print = ILI9341_print,
    .make_new = ILI9341_make_new,
    .locals_dict = (mp_obj_dict_t*)&ILI9341_locals_dict,
};

STATIC mp_obj_t ILI9341_make_new(const mp_obj_type_t *type,
                                 size_t n_args,
                                 size_t n_kw,
                                 const mp_obj_t *all_args)
{
    enum{
         ARG_mhz,
         ARG_spihost,
         ARG_miso,
         ARG_mosi,
         ARG_clk,
         ARG_cs,
         ARG_dc,
         ARG_rst,
         ARG_backlight,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mhz,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=40}},
        { MP_QSTR_spihost,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=HSPI_HOST}},
        { MP_QSTR_miso,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},             
        { MP_QSTR_mosi,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
        { MP_QSTR_clk,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
        { MP_QSTR_cs,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
        { MP_QSTR_dc,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
        { MP_QSTR_rst,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
        { MP_QSTR_backlight,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    ILI9341_t *self = m_new_obj(ILI9341_t);
    self->base.type = type;
    self->spi = NULL;
    self->mhz = args[ARG_mhz].u_int;
    self->spihost = args[ARG_spihost].u_int;
    self->miso = args[ARG_miso].u_int;
    self->mosi = args[ARG_mosi].u_int;
    self->clk = args[ARG_clk].u_int;
    self->cs = args[ARG_cs].u_int;
    self->dc = args[ARG_dc].u_int;
    self->rst = args[ARG_rst].u_int;
    self->backlight = args[ARG_backlight].u_int;

    return MP_OBJ_FROM_PTR(self);
}


STATIC const mp_rom_map_elem_t ILI9341_globals_table[] = {
        { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ILI9341) },
        { MP_ROM_QSTR(MP_QSTR_display), (mp_obj_t)&ILI9341_type},
};
         

STATIC MP_DEFINE_CONST_DICT (
    mp_module_ILI9341_globals,
    ILI9341_globals_table
);

const mp_obj_module_t mp_module_ILI9341 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_ILI9341_globals
};

//////////////////////////////////////////////////////////////////////////////
// ILI9341 driver implementation
//////////////////////////////////////////////////////////////////////////////

STATIC void disp_spi_init(ILI9341_t *self)
{
	esp_err_t ret;

	spi_bus_config_t buscfg={
		.miso_io_num=self->miso,
		.mosi_io_num=self->mosi,
		.sclk_io_num=self->clk,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=128*1024,
	};

	spi_device_interface_config_t devcfg={
		.clock_speed_hz=self->mhz*1000*1000, //Clock out at DISP_SPI_MHZ MHz
		.mode=0,                             //SPI mode 0
		.spics_io_num=self->cs,              //CS pin
		.queue_size=1,
		.pre_cb=NULL,
		.post_cb=NULL,
		.flags=SPI_DEVICE_HALFDUPLEX,
		.duty_cycle_pos=128,
	};

    gpio_pad_select_gpio(self->miso);
    gpio_pad_select_gpio(self->mosi);
    gpio_pad_select_gpio(self->clk);

    gpio_set_direction(self->miso, GPIO_MODE_INPUT);
    gpio_set_pull_mode(self->miso, GPIO_PULLUP_ONLY);
    gpio_set_direction(self->mosi, GPIO_MODE_OUTPUT);
    gpio_set_direction(self->clk, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(self->cs);
	//Initialize the SPI bus
	ret=spi_bus_initialize(self->spihost, &buscfg, 1);
    if (ret != ESP_OK) nlr_raise(
        mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed initializing SPI bus"));

	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi);
    if (ret != ESP_OK) nlr_raise(
        mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));
}

STATIC void disp_spi_send(ILI9341_t *self, const uint8_t * data, uint16_t length)
{
	if (length == 0) return;           //no need to send anything

	spi_transaction_t t;
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = length * 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = data;              //Data
//	esp_err_t ret;
//	ret=spi_device_transmit(spi, &t);  //Transmit!
//	assert(ret==ESP_OK);            	 //Should have had no issues
	spi_device_queue_trans(self->spi, &t, portMAX_DELAY);

	spi_transaction_t * rt;
	spi_device_get_trans_result(self->spi, &rt, portMAX_DELAY);
}

STATIC void ili9441_send_cmd(ILI9341_t *self, uint8_t cmd)
{
	gpio_set_level(self->dc, 0);	 /*Command mode*/
	disp_spi_send(self, &cmd, 1);
}

STATIC void ili9341_send_data(ILI9341_t *self, const void * data, uint16_t length)
{
	gpio_set_level(self->dc, 1);	 /*Data mode*/
	disp_spi_send(self, data, length);
}

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[34];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

STATIC const lcd_init_cmd_t ili_init_cmds[]={
    {HX8357_SWRESET, {20}, 0x80}, // Soft reset, then delay 10 ms
    {HX8357D_SETC, {0xFF, 0x83, 0x57}, 3},
    {0xFF, {100}, 0x80},          // No command, just delay 300 ms
    {HX8357_SETRGB, {0x80, 0x00, 0x06, 0x06}, 4},    // 0x80 enables SDO pin (0x00 disables)
    {HX8357D_SETCOM, {0x25}, 1},                      // -1.52V
    {HX8357_SETOSC, {0x68}, 1},                      // Normal mode 70Hz, Idle mode 55 Hz
    {HX8357_SETPANEL, {0x00}, 1},                      // 
    {HX8357_SETPWR1, {
      0x00,                  // Not deep standby
      0x15,                      // BT
      0x1C,                      // VSPR
      0x1C,                      // VSNR
      0x83,                      // AP
      0xAA}, 6},                      // FS
    {HX8357D_SETSTBA, {
      0x50,                      // OPON normal
      0x50,                      // OPON idle
      0x01,                      // STBA
      0x3C,                      // STBA
      0x1E,                      // STBA
      0x08}, 6},                      // GEN
    {HX8357D_SETCYC, {
      0x02,                      // NW 0x02
      0x40,                      // RTN
      0x00,                      // DIV
      0x2A,                      // DUM
      0x2A,                      // DUM
      0x0D,                      // GDON
      0x78}, 7},                      // GDOFF
    {HX8357D_SETGAMMA, {
      0x02, 0x0A, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b,
      0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A,
      0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3A,
      0x27, 0x1B, 0x08, 0x09, 0x03, 0x00, 0x01}, 34},
    {0x53, {0x04}, 1},
    {HX8357_COLMOD, {0x55}, 1},                      // 16 bit
    {HX8357_MADCTL, {0xC0}, 1},
    {HX8357_TEON, {0x00}, 1},                      // TW off
    {HX8357_TEARLINE, {0x00, 0x02}, 2},
    {HX8357_SLPOUT, {30}, 0x80}, // Exit Sleep, then delay 150 ms
    {HX8357_MADCTL, {0xe8}, 1},
    {HX8357_DISPON, {10}, 0x80}, // Main screen turn on, delay 50 ms
    {0, {0}, 0xff},                           // END OF COMMAND LIST
  };

STATIC mp_obj_t mp_init_ILI9341(mp_obj_t self_in)
{
    ILI9341_t *self = MP_OBJ_TO_PTR(self_in);
    mp_activate_ILI9341(self_in);

    disp_spi_init(self);
    gpio_pad_select_gpio(self->dc);
    gpio_pad_select_gpio(self->backlight);

	//Initialize non-SPI GPIOs
	gpio_set_direction(self->dc, GPIO_MODE_OUTPUT);
	gpio_set_direction(self->backlight, GPIO_MODE_OUTPUT);

	// printf("ILI9341 initialization.\n");


	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
                if (ili_init_cmds[cmd].cmd !=0xff) {
		ili9441_send_cmd(self, ili_init_cmds[cmd].cmd);
		}
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(ili_init_cmds[cmd].data[0]);
		} else {
			ili9341_send_data(self, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes & 0x1F);
		}
		cmd++;
	}

	///Enable backlight
	printf("Enable backlight.\n");
	gpio_set_level(self->backlight, 1);
    return mp_const_none;
}


STATIC void ili9431_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{

	uint8_t data[4];

 	ILI9341_t *self = g_ILI9341;

	/*Column addresses*/
	ili9441_send_cmd(self, HX8357_CASET);
	data[0] = (area->x1 >> 8) & 0xFF;
	data[1] = area->x1 & 0xFF;
	data[2] = (area->x2 >> 8) & 0xFF;
	data[3] = area->x2 & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Page addresses*/
	ili9441_send_cmd(self, HX8357_PASET);
	data[0] = (area->y1 >> 8) & 0xFF;
	data[1] = area->y1 & 0xFF;
	data[2] = (area->y2 >> 8) & 0xFF;
	data[3] = area->y2 & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Memory write*/
	ili9441_send_cmd(self, HX8357_RAMWR);

	uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
	
	/*Byte swapping is required*/
	/*
	uint32_t i;
	uint8_t * color_u8 = (uint8_t *) color_p;
	uint8_t color_tmp;
	for(i = 0; i < size * 2; i += 2) {
		color_tmp = color_u8[i + 1];
		color_u8[i + 1] = color_u8[i];
		color_u8[i] = color_tmp;
		//printf("%d", color_tmp);
	}
	*/
	ili9341_send_data(self, (void*)color_p, size * 2);

	/*
	while(size > LV_HOR_RES) {
		ili9341_send_data((void*)color_p, LV_HOR_RES * 2);
		//vTaskDelay(10 / portTICK_PERIOD_MS);
		size -= LV_HOR_RES;
		color_p += LV_HOR_RES;
	}
	ili9341_send_data((void*)color_p, size * 2);	*/ /*Send the remaining data*/

	lv_disp_flush_ready(disp_drv);
}
