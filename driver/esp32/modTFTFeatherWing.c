/**
 * Includes
 **/
#include "../include/common.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lvgl/src/lv_hal/lv_hal_disp.h"

/**
 * ILI9341 requires specific lv_conf resolution and color depth
 **/
#if LV_COLOR_DEPTH != 16
#error "modILI9341: LV_COLOR_DEPTH must be set to 16!"
#endif

/**
 * HX8357 Defines
 **/
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
// them. Some additional ones are in the HX8357 lib -- add all in GFX!
// Color definitions
#define	HX8357_BLACK   0x0000 ///< BLACK color for drawing graphics
#define	HX8357_BLUE    0x001F ///< BLUE color for drawing graphics
#define	HX8357_RED     0xF800 ///< RED color for drawing graphics
#define	HX8357_GREEN   0x07E0 ///< GREEN color for drawing graphics
#define HX8357_CYAN    0x07FF ///< CYAN color for drawing graphics
#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
#define HX8357_YELLOW  0xFFE0 ///< YELLOW color for drawing graphics
#define HX8357_WHITE   0xFFFF ///< WHITE color for drawing graphics

/**
 * STMPE610 Defines
 **/
/** STMPE610 Address **/
#define STMPE_ADDR 0x41

/** Reset Control **/
#define STMPE_SYS_CTRL1 0x03
#define STMPE_SYS_CTRL1_RESET 0x02

/** Clock Contrl **/
#define STMPE_SYS_CTRL2 0x04

/** Touchscreen controller setup **/
#define STMPE_TSC_CTRL 0x40
#define STMPE_TSC_CTRL_EN 0x01
#define STMPE_TSC_CTRL_XYZ 0x00
#define STMPE_TSC_CTRL_XY 0x02

/** Interrupt control **/
#define STMPE_INT_CTRL 0x09
#define STMPE_INT_CTRL_POL_HIGH 0x04
#define STMPE_INT_CTRL_POL_LOW 0x00
#define STMPE_INT_CTRL_EDGE 0x02
#define STMPE_INT_CTRL_LEVEL 0x00
#define STMPE_INT_CTRL_ENABLE 0x01
#define STMPE_INT_CTRL_DISABLE 0x00

/** Interrupt enable **/
#define STMPE_INT_EN 0x0A
#define STMPE_INT_EN_TOUCHDET 0x01
#define STMPE_INT_EN_FIFOTH 0x02
#define STMPE_INT_EN_FIFOOF 0x04
#define STMPE_INT_EN_FIFOFULL 0x08
#define STMPE_INT_EN_FIFOEMPTY 0x10
#define STMPE_INT_EN_ADC 0x40
#define STMPE_INT_EN_GPIO 0x80

/** Interrupt status **/
#define STMPE_INT_STA 0x0B
#define STMPE_INT_STA_TOUCHDET 0x01

/** ADC control **/
#define STMPE_ADC_CTRL1 0x20
#define STMPE_ADC_CTRL1_12BIT 0x08
#define STMPE_ADC_CTRL1_10BIT 0x00

/** ADC control **/
#define STMPE_ADC_CTRL2 0x21
#define STMPE_ADC_CTRL2_1_625MHZ 0x00
#define STMPE_ADC_CTRL2_3_25MHZ 0x01
#define STMPE_ADC_CTRL2_6_5MHZ 0x02

/** Touchscreen controller configuration **/
#define STMPE_TSC_CFG 0x41
#define STMPE_TSC_CFG_1SAMPLE 0x00
#define STMPE_TSC_CFG_2SAMPLE 0x40
#define STMPE_TSC_CFG_4SAMPLE 0x80
#define STMPE_TSC_CFG_8SAMPLE 0xC0
#define STMPE_TSC_CFG_DELAY_10US 0x00
#define STMPE_TSC_CFG_DELAY_50US 0x08
#define STMPE_TSC_CFG_DELAY_100US 0x10
#define STMPE_TSC_CFG_DELAY_500US 0x18
#define STMPE_TSC_CFG_DELAY_1MS 0x20
#define STMPE_TSC_CFG_DELAY_5MS 0x28
#define STMPE_TSC_CFG_DELAY_10MS 0x30
#define STMPE_TSC_CFG_DELAY_50MS 0x38
#define STMPE_TSC_CFG_SETTLE_10US 0x00
#define STMPE_TSC_CFG_SETTLE_100US 0x01
#define STMPE_TSC_CFG_SETTLE_500US 0x02
#define STMPE_TSC_CFG_SETTLE_1MS 0x03
#define STMPE_TSC_CFG_SETTLE_5MS 0x04
#define STMPE_TSC_CFG_SETTLE_10MS 0x05
#define STMPE_TSC_CFG_SETTLE_50MS 0x06
#define STMPE_TSC_CFG_SETTLE_100MS 0x07

/** FIFO level to generate interrupt **/
#define STMPE_FIFO_TH 0x4A

/** Current filled level of FIFO **/
#define STMPE_FIFO_SIZE 0x4C

/** Current status of FIFO **/
#define STMPE_FIFO_STA 0x4B
#define STMPE_FIFO_STA_RESET 0x01
#define STMPE_FIFO_STA_OFLOW 0x80
#define STMPE_FIFO_STA_FULL 0x40
#define STMPE_FIFO_STA_EMPTY 0x20
#define STMPE_FIFO_STA_THTRIG 0x10

/** Touchscreen controller drive I **/
#define STMPE_TSC_I_DRIVE 0x58
#define STMPE_TSC_I_DRIVE_20MA 0x00
#define STMPE_TSC_I_DRIVE_50MA 0x01

/** Data port for TSC data address **/
#define STMPE_TSC_DATA_X 0x4D
#define STMPE_TSC_DATA_Y 0x4F
#define STMPE_TSC_FRACTION_Z 0x56
#define STMPE_TSC_DATA_XYZ 0xD7

/** GPIO **/
#define STMPE_GPIO_SET_PIN 0x10
#define STMPE_GPIO_CLR_PIN 0x11
#define STMPE_GPIO_DIR 0x13
#define STMPE_GPIO_ALT_FUNCT 0x17

//////////////////////////////////////////////////////////////////////////////
// Module definition
//////////////////////////////////////////////////////////////////////////////
typedef struct {
   mp_obj_base_t base;
  
   spi_device_handle_t spi_tft;
   spi_device_handle_t spi_ts;
   spi_device_handle_t spi_sd;

   uint8_t spihost;
   uint8_t miso;
   uint8_t mosi;
   uint8_t clk;

   uint8_t tft_mhz;
   uint8_t tcs;
   uint8_t dc;
   uint8_t rst;
   uint8_t backlight;

   uint8_t ts_mhz;
   uint8_t rcs;
   uint8_t irq;
   

   uint8_t sd_mhz;
   uint8_t scs;
  
} TFTFeatherWing_obj_t;

// Unfortunately, lvgl doesnt pass user_data to callbacks, so we use this global.
// This means we can have only one active display driver instance, pointed by this global.
STATIC TFTFeatherWing_obj_t *g_TFTFeatherWing = NULL;

/**
 * Base Function & Function Accessible to MP Protopype
 **/
STATIC mp_obj_t TFTFeatherWing_make_new(const mp_obj_type_t *type,
					size_t n_args,
					size_t n_kw,
					const mp_obj_t *all_args);
STATIC mp_obj_t mp_init_TFTFeatherWing(mp_obj_t self_in);
STATIC mp_obj_t mp_activate_TFTFeatherWing(mp_obj_t self_in);

STATIC void tft_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_init_TFTFeatherWing_obj, mp_init_TFTFeatherWing);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_activate_TFTFeatherWing_obj, mp_activate_TFTFeatherWing);
DEFINE_PTR_OBJ(tft_flush);

// TFTFeatherWing Class and methods
STATIC const mp_rom_map_elem_t TFTFeatherWing_locals_dict_table[] = {
   { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_init_TFTFeatherWing_obj) },
   { MP_ROM_QSTR(MP_QSTR_activate), MP_ROM_PTR(&mp_activate_TFTFeatherWing_obj) },
   { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&PTR_OBJ(tft_flush)) },
};

STATIC MP_DEFINE_CONST_DICT(TFTFeatherWing_locals_dict, TFTFeatherWing_locals_dict_table);

STATIC const mp_obj_type_t TFTFeatherWing_obj_type = {
   { &mp_type_type },
   .name = MP_QSTR_TFTFeatherWing,
   //.print = TFTFeatherWing_print,
   .make_new = TFTFeatherWing_make_new,
   .locals_dict = (mp_obj_dict_t*)&TFTFeatherWing_locals_dict,
};

// Creating Python Module
STATIC const mp_rom_map_elem_t TFTFeatherWing_globals_table[] = {
   { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_TFTFeatherWing) },
   { MP_ROM_QSTR(MP_QSTR_TFTFeatherWing), (mp_obj_t)&TFTFeatherWing_obj_type},
};
         

STATIC MP_DEFINE_CONST_DICT (
   mp_module_TFTFeatherWing_globals,
   TFTFeatherWing_globals_table
   );

const mp_obj_module_t mp_module_TFTFeatherWing = {
   .base = { &mp_type_module },
   .globals = (mp_obj_dict_t*)&mp_module_TFTFeatherWing_globals
};

/**
 * FTFFeatherWing driver implementation
 **/

/**
 * Common Function Prototypr
 **/
STATIC void spi_bus_init(TFTFeatherWing_obj_t *self);

/**
 * TS Function Prototype
 */
STATIC void ts_init(TFTFeatherWing_obj_t *self);
STATIC uint8_t ts_read_register_byte(TFTFeatherWing_obj_t *self, const uint8_t reg);
STATIC void ts_write_register_byte(TFTFeatherWing_obj_t *self, const uint8_t reg, const uint8_t val);
STATIC void ts_write_byte(TFTFeatherWing_obj_t *self, const uint8_t val);

/**
 * TFT Function Prototype
 */
STATIC void tft_init(TFTFeatherWing_obj_t *self);
STATIC void tft_write(TFTFeatherWing_obj_t *self, const uint8_t * data, const uint16_t length);
STATIC void tft_send_cmd(TFTFeatherWing_obj_t *self, uint8_t cmd);
STATIC void tft_send_data(TFTFeatherWing_obj_t *self, const void * data, uint16_t length);

/**
 * Base Function & Function Accessible to MP Protopype
 **/
STATIC mp_obj_t TFTFeatherWing_make_new(const mp_obj_type_t *type,
					size_t n_args,
					size_t n_kw,
					const mp_obj_t *all_args)
{
   enum{      
      ARG_spihost,
	
      ARG_miso,
      ARG_mosi,
      ARG_clk,

      ARG_tft_mhz,
      ARG_tcs,
      ARG_dc,
      ARG_rst,
      ARG_backlight,

      ARG_ts_mhz,
      ARG_rcs,
      ARG_irq,

      ARG_sd_mhz,
      ARG_scs,
   };

   static const mp_arg_t allowed_args[] = {      
      { MP_QSTR_spihost,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=HSPI_HOST}},
	
      { MP_QSTR_miso,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=19}},
      { MP_QSTR_mosi,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=18}},
      { MP_QSTR_clk,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=5}},

      
      { MP_QSTR_tft_mhz,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=24}},
      { MP_QSTR_tcs,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=15}},
      { MP_QSTR_dc,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=33}},
      { MP_QSTR_rst,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
      { MP_QSTR_backlight,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},

      
      { MP_QSTR_ts_mhz,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=1}},
      { MP_QSTR_rcs,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=32}},
      { MP_QSTR_irq,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=-1}},
      
      { MP_QSTR_sd_mhz,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=1}},
      { MP_QSTR_scs,MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int=14}},
   };

   mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
   mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
   TFTFeatherWing_obj_t *self = m_new_obj(TFTFeatherWing_obj_t);
   
   self->base.type = type;
   self->spi_ts = NULL;
   self->spi_tft = NULL;
   self->spi_sd = NULL;

   self->spihost = args[ARG_spihost].u_int;
   
   self->miso = args[ARG_miso].u_int;
   self->mosi = args[ARG_mosi].u_int;
   self->clk = args[ARG_clk].u_int;

   self->tft_mhz = args[ARG_tft_mhz].u_int;
   self->tcs = args[ARG_tcs].u_int;
   self->dc = args[ARG_dc].u_int;
   self->rst = args[ARG_rst].u_int;
   self->backlight = args[ARG_backlight].u_int;
   
   self->ts_mhz = args[ARG_ts_mhz].u_int;
   self->rcs = args[ARG_rcs].u_int;
   self->irq = args[ARG_irq].u_int;
   
   self->sd_mhz = args[ARG_sd_mhz].u_int;
   self->irq = args[ARG_irq].u_int;
   
   return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_activate_TFTFeatherWing(mp_obj_t self_in) {
   TFTFeatherWing_obj_t *self = MP_OBJ_TO_PTR(self_in);
   g_TFTFeatherWing = self;
   return mp_const_none;
}

STATIC mp_obj_t mp_init_TFTFeatherWing(mp_obj_t self_in) {
   TFTFeatherWing_obj_t *self = MP_OBJ_TO_PTR(self_in);
   mp_activate_TFTFeatherWing(self_in);
     
   spi_bus_init(self);
   ts_init(self);
   tft_init(self);
   
   return mp_const_none;
}

STATIC void tft_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
   //printf("Flush\n");
   uint8_t data[4];

   TFTFeatherWing_obj_t *self = g_TFTFeatherWing;

   /*Column addresses*/
   tft_send_cmd(self, HX8357_CASET);
   data[0] = (area->x1 >> 8) & 0xFF;
   data[1] = area->x1 & 0xFF;
   data[2] = (area->x2 >> 8) & 0xFF;
   data[3] = area->x2 & 0xFF;
   tft_send_data(self, data, 4);

   /*Page addresses*/
   tft_send_cmd(self, HX8357_PASET);
   data[0] = (area->y1 >> 8) & 0xFF;
   data[1] = area->y1 & 0xFF;
   data[2] = (area->y2 >> 8) & 0xFF;
   data[3] = area->y2 & 0xFF;
   tft_send_data(self, data, 4);

   /*Memory write*/
   tft_send_cmd(self, HX8357_RAMWR);

   uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);

   tft_send_data(self, (void*)color_p, size * 2);

   lv_disp_flush_ready(disp_drv);
}

/**
 * Common Function
 **/
STATIC void spi_bus_init(TFTFeatherWing_obj_t *self) {
   esp_err_t ret;

   //Initialize the SPI bus
   spi_bus_config_t buscfg={
      .miso_io_num=self->miso,
      .mosi_io_num=self->mosi,
      .sclk_io_num=self->clk,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
      .max_transfer_sz=128*1024,
   };

   ret=spi_bus_initialize(self->spihost, &buscfg, 1);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed initializing SPI bus"));
   }
}

/**
 * TS Function
 */
STATIC void ts_init(TFTFeatherWing_obj_t *self) {
   esp_err_t ret;
   
   //Attach the Touch Screen to the SPI bus
   spi_device_interface_config_t devcfg={
      .clock_speed_hz=self->5000, //Clock out at DISP_SPI_MHZ MHz
      .mode=0,                             //SPI mode 0
      .spics_io_num=self->rcs,              //CS pin
      .queue_size=1,
      .pre_cb=NULL,
      .post_cb=NULL,
      .flags=SPI_DEVICE_HALFDUPLEX,
      .duty_cycle_pos=128,
      .command_bits=8,
   };

   //gpio_pad_select_gpio(self->rcs);
   //gpio_set_direction(self->rcs, GPIO_MODE_OUTPUT);
   
   ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi_ts);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));
   }
   
   uint16_t ts_version;
   ts_version = ts_read_register_byte(self, 0);
   ts_version <<= 8;
   ts_version |= ts_read_register_byte(self, 1);
   printf("TS Version %x\n", ts_version);

   // Initialize STMPE610
   ts_write_register_byte(self, STMPE_SYS_CTRL2, 0x0); // turn on clocks!
   ts_write_register_byte(self, STMPE_TSC_CTRL,
			  STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
   ts_write_register_byte(self, STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
   ts_write_register_byte(self, STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT |
			  (0x6 << 4)); // 96 clocks per conversion
   ts_write_register_byte(self, STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
   ts_write_register_byte(self, STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE |
			  STMPE_TSC_CFG_DELAY_1MS |
			  STMPE_TSC_CFG_SETTLE_5MS);
   ts_write_register_byte(self, STMPE_TSC_FRACTION_Z, 0x6);
   ts_write_register_byte(self, STMPE_FIFO_TH, 1);
   ts_write_register_byte(self, STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
   ts_write_register_byte(self, STMPE_FIFO_STA, 0); // unreset
   ts_write_register_byte(self, STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
   ts_write_register_byte(self, STMPE_INT_STA, 0xFF); // reset all ints
   ts_write_register_byte(self, STMPE_INT_CTRL,
			  STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);
}

STATIC uint8_t ts_read_register_byte(TFTFeatherWing_obj_t *self, const uint8_t reg) {
   printf("Read TS register\n");
   esp_err_t ret;
 
   spi_transaction_t t;
   uint8_t read_data[2];

   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.cmd = (reg | 0x80);
   printf("CMD %x\n", t.cmd);
   t.rxlength = 16;              //Length is in bytes, transaction length is in bits.
   t.rx_buffer = read_data;

   spi_device_queue_trans(self->spi_ts, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   ret=spi_device_get_trans_result(self->spi_ts, &rt, portMAX_DELAY);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Transation"));
   }
   printf("Read Data: %x %x\n", read_data[0], read_data[1]);

   return read_data[0];
}

STATIC void ts_write_register_byte(TFTFeatherWing_obj_t *self, const uint8_t reg, const uint8_t val) {
   printf("Write TS register\n");
   esp_err_t ret;
 
   spi_transaction_t t;
   uint8_t write_data[1];

   write_data[0] = val;
   
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.cmd = reg;
   t.length = 8;              //Length is in bytes, transaction length is in bits.
   t.tx_buffer = write_data;

   spi_device_queue_trans(self->spi_ts, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   ret=spi_device_get_trans_result(self->spi_ts, &rt, portMAX_DELAY);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Transation"));
   }
}

STATIC void ts_write_byte(TFTFeatherWing_obj_t *self, const uint8_t val) {
   printf("Write TS\n");
   esp_err_t ret;
 
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.cmd = val;
   spi_device_queue_trans(self->spi_ts, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   ret=spi_device_get_trans_result(self->spi_ts, &rt, portMAX_DELAY);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Transation"));
   }
}

/**
 * TFT Function
 */
/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
   uint8_t cmd;
   uint8_t data[34];
   uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

STATIC void tft_init(TFTFeatherWing_obj_t *self) {
    esp_err_t ret;
   
   //Attach the TFT to the SPI bus
   spi_device_interface_config_t devcfg={
      .clock_speed_hz=self->tft_mhz*1000*1000, //Clock out at DISP_SPI_MHZ MHz
      .mode=0,                             //SPI mode 0
      .spics_io_num=self->tcs,              //CS pin
      .queue_size=1,
      .pre_cb=NULL,
      .post_cb=NULL,
      .flags=SPI_DEVICE_HALFDUPLEX,
      .duty_cycle_pos=128,
   };
   
   ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi_tft);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));
   }

   gpio_pad_select_gpio(self->dc);
   gpio_set_direction(self->dc, GPIO_MODE_OUTPUT);   
   gpio_set_level(self->dc, 0);
   
   //Initialize non-SPI GPIOs   
   const lcd_init_cmd_t hx_init_cmds[]={
      {HX8357_SWRESET, {10}, 0x80}, // Soft reset, then delay 10 ms
      {HX8357D_SETC, {0xFF, 0x83, 0x57}, 3},
      {0xFF, {200}, 0x80},          // No command, just delay 200 ms
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
      {HX8357_SLPOUT, {150}, 0x80}, // Exit Sleep, then delay 150 ms
      {HX8357_MADCTL, {0xe8}, 1},
      {HX8357_DISPON, {50}, 0x80}, // Main screen turn on, delay 50 ms
      {0, {0}, 0xff},                           // END OF COMMAND LIST
   };
   
   //Send all the commands
   uint16_t cmd = 0;
   while (hx_init_cmds[cmd].databytes!=0xff) {
      if (hx_init_cmds[cmd].cmd !=0xff) {
	 tft_send_cmd(self, hx_init_cmds[cmd].cmd);
      }
      if (hx_init_cmds[cmd].databytes & 0x80) {
	 vTaskDelay(hx_init_cmds[cmd].data[0]/portTICK_PERIOD_MS);
      } else {
	 tft_send_data(self, hx_init_cmds[cmd].data, hx_init_cmds[cmd].databytes & 0x1F);
      }
      cmd++;
   }
   tft_send_cmd(self, 0x00);
   printf("End TFT Init\n");
}

STATIC void tft_write(TFTFeatherWing_obj_t *self, const uint8_t * data, const uint16_t length) {
   if (length == 0) {
      return;           //no need to send anything
   }
   
   esp_err_t ret;
   
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.length = length * 8;              //Length is in bytes, transaction length is in bits.
   t.tx_buffer = data;              //Data
   
   spi_device_queue_trans(self->spi_tft, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   ret = spi_device_get_trans_result(self->spi_tft, &rt, portMAX_DELAY);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));
   }
}

STATIC void tft_send_cmd(TFTFeatherWing_obj_t *self, const uint8_t cmd) {
   gpio_set_level(self->dc, 0);	 /*Command mode*/
   tft_write(self, &cmd, 1);
   gpio_set_level(self->dc, 0);
}

STATIC void tft_send_data(TFTFeatherWing_obj_t *self, const void * data, const uint16_t length) {
   gpio_set_level(self->dc, 1);	 // Data mode
   tft_write(self, data, length);
   gpio_set_level(self->dc, 0);
}
