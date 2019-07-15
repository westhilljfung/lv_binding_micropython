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

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_init_TFTFeatherWing_obj, mp_init_TFTFeatherWing);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_activate_TFTFeatherWing_obj, mp_activate_TFTFeatherWing);

// TFTFeatherWing Class and methods
STATIC const mp_rom_map_elem_t TFTFeatherWing_locals_dict_table[] = {
   { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_init_TFTFeatherWing_obj) },
   { MP_ROM_QSTR(MP_QSTR_activate), MP_ROM_PTR(&mp_activate_TFTFeatherWing_obj) },
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
 * Base Function & Function Accessible to MP Protopype
 **/
STATIC mp_obj_t TFTFeatherWing_make_new(const mp_obj_type_t *type,
					size_t n_args,
					size_t n_kw,
					const mp_obj_t *all_args)
{
   static const mp_arg_t allowed_args[] = {    };

   mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
   mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
   TFTFeatherWing_obj_t *self = m_new_obj(TFTFeatherWing_obj_t);
   
   self->base.type = type;
   self->spi_ts = NULL;
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
     
   esp_err_t ret;
   
   //Initialize the SPI bus
   spi_bus_config_t buscfg={
      .miso_io_num=19,
      .mosi_io_num=18,
      .sclk_io_num=5,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
   };

   ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
   ESP_ERROR_CHECK(ret);

   spi_device_handle_t spi;
   //Attach the Touch Screen to the SPI bus
   spi_device_interface_config_t devcfg_ts={
      .clock_speed_hz=1000000, //Clock out at 1 MHz
      .mode=0,                             //SPI mode 0
      .spics_io_num=32,              //CS pin
      .queue_size=1,
   };

   ret=spi_bus_add_device(VSPI_HOST, &devcfg_ts, spi);
   ESP_ERROR_CHECK(ret);
   
   spi_transaction_t t;
   uint8_t read_data[4];
   uint8_t write_data[4];
   uint16_t i;

   for (i = 0; i < 4; i++) {
      write_data[i] = 0x82;
   }
   
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   
   t.length = 32;
   t.tx_buffer = write_data;
   t.rx_buffer = read_data;

   spi_device_queue_trans(spi, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   ret=spi_device_get_trans_result(spi, &rt, portMAX_DELAY);
   ESP_ERROR_CHECK(ret);
   
   printf("Read Data: %x %x %x %x\n", read_data[0], read_data[1], read_data[2], read_data[3]);

   return mp_const_none;
}
