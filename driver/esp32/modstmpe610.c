//////////////////////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////////////////////

#include "../include/common.h"
#include "lvgl/src/lv_hal/lv_hal_indev.h"
//#include "lvgl/src/lv_core/lv_disp.h"
//#include "py/obj.h"
//#include "py/runtime.h"
//#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

//////////////////////////////////////////////////////////////////////////////
// Defines
//////////////////////////////////////////////////////////////////////////////
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

typedef struct _stmpe610_obj_t {
   mp_obj_base_t base;

   uint8_t mhz;
   uint8_t spihost;
   uint8_t cs;
   uint8_t irq;

   int16_t x_min;
   int16_t y_min;
   int16_t x_max;
   int16_t y_max;
   bool x_inv;
   bool y_inv;    
   bool xy_swap;

   spi_device_handle_t spi;

} stmpe610_obj_t;

// Unfortunately, lvgl doesn't pass user_data to callbacks, so we use this global.
// This means we can have only one active touch driver instance, pointed by this global.
STATIC stmpe610_obj_t *g_STMPE610 = NULL;

STATIC mp_obj_t stmpe610_make_new(const mp_obj_type_t *type,
				  size_t n_args,
				  size_t n_kw,
				  const mp_obj_t *all_args);

STATIC mp_obj_t mp_stmpe610_init(mp_obj_t self_in);

STATIC mp_obj_t mp_activate_stmpe610(mp_obj_t self_in) {
   stmpe610_obj_t *self = MP_OBJ_TO_PTR(self_in);
   g_STMPE610 = self;
   return mp_const_none;
}

STATIC bool stmpe610_read(lv_indev_data_t *data);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_init_stmpe610_obj, mp_stmpe610_init);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_activate_stmpe610_obj, mp_activate_stmpe610);
DEFINE_PTR_OBJ(stmpe610_read);

STATIC const mp_rom_map_elem_t stmpe610_locals_dict_table[] = {
   { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_init_stmpe610_obj) },
   { MP_ROM_QSTR(MP_QSTR_activate), MP_ROM_PTR(&mp_activate_stmpe610_obj) },
   { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&PTR_OBJ(stmpe610_read)) },
};


STATIC MP_DEFINE_CONST_DICT(stmpe610_locals_dict, stmpe610_locals_dict_table);

STATIC const mp_obj_type_t stmpe610_type = {
   { &mp_type_type },
   .name = MP_QSTR_stmpe610,
   //.print = stmpe610_print,
   .make_new = stmpe610_make_new,
   .locals_dict = (mp_obj_dict_t*)&stmpe610_locals_dict,
};

STATIC mp_obj_t stmpe610_make_new(const mp_obj_type_t *type,
				  size_t n_args,
				  size_t n_kw,
				  const mp_obj_t *all_args) {
   enum{
      ARG_mhz,
      ARG_spihost,
      ARG_cs,
      ARG_irq,

      ARG_x_min,
      ARG_y_min,
      ARG_x_max,
      ARG_y_max,
      ARG_x_inv,
      ARG_y_inv,
      ARG_xy_swap,
   };

   static const mp_arg_t allowed_args[] = {
      { MP_QSTR_mhz, MP_ARG_INT, {.u_int = 20}},
      { MP_QSTR_spihost, MP_ARG_INT, {.u_int = HSPI_HOST}},
      { MP_QSTR_cs, MP_ARG_INT, {.u_int = 33}},
      { MP_QSTR_irq, MP_ARG_INT, {.u_int = 25}},

      { MP_QSTR_x_min, MP_ARG_INT, {.u_int = 1000}},
      { MP_QSTR_y_min, MP_ARG_INT, {.u_int = 1000}},
      { MP_QSTR_x_max, MP_ARG_INT, {.u_int = 3200}},
      { MP_QSTR_y_max, MP_ARG_INT, {.u_int = 2000}},
      { MP_QSTR_x_inv, MP_ARG_BOOL, {.u_obj = mp_const_true}},
      { MP_QSTR_y_inv, MP_ARG_BOOL, {.u_obj = mp_const_true}},
      { MP_QSTR_xy_swap, MP_ARG_BOOL, {.u_obj = mp_const_false}},
   };

   mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
   mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
   stmpe610_obj_t *self = m_new_obj(stmpe610_obj_t);
   self->base.type = type;

   self->mhz = args[ARG_mhz].u_int;
   self->spihost = args[ARG_spihost].u_int;
   self->cs = args[ARG_cs].u_int;
   self->irq = args[ARG_irq].u_int;

   self->x_min = args[ARG_x_min].u_int;
   self->y_min = args[ARG_y_min].u_int;
   self->x_max = args[ARG_x_max].u_int;
   self->y_max = args[ARG_y_max].u_int;
   self->x_inv = args[ARG_x_inv].u_bool;
   self->y_inv = args[ARG_y_inv].u_bool;
   self->xy_swap = args[ARG_xy_swap].u_bool;
   return MP_OBJ_FROM_PTR(self);
}

STATIC const mp_rom_map_elem_t stmpe610_globals_table[] = {
   { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_STMPE610) },
   { MP_ROM_QSTR(MP_QSTR_stmpe610), (mp_obj_t)&STMPE610_type},
};
         

STATIC MP_DEFINE_CONST_DICT (
   mp_module_stmpe610_globals,
   stmpe610_globals_table
   );

const mp_obj_module_t mp_module_STMPE610 = {
   .base = { &mp_type_module },
   .globals = (mp_obj_dict_t*)&mp_module_stmpe610_globals
};

//////////////////////////////////////////////////////////////////////////////
// Module implementation
//////////////////////////////////////////////////////////////////////////////
STATIC void write_register(stmpe610_obj_t *self, const uint8_t reg, const uint8_t val) {
   unit8_t data[2];
   data[0] = reg;
   data[1] = val;
  
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.length = 2 * 8;              //Length is in bytes, transaction length is in bits.
   t.tx_buffer = data;

   spi_device_queue_trans(self->spi, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   spi_device_get_trans_result(self->spi, &rt, portMAX_DELAY);
}


STATIC uint8_t read_register_byte(stmpe610_obj_t *self, const uint8_t reg) {
   uint8_t data_read;
   unit8_t data[2];
   data[0] = 0x80 | reg;
   data[1] = 0x00;
   data_read = 0;
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));		//Zero out the transaction
   t.length = 8;              //Length is in bytes, transaction length is in bits.

   t.tx_buffer = data;
   t.rx_buffer = &data_read;

   spi_device_queue_trans(self->spi, &t, portMAX_DELAY);

   spi_transaction_t * rt;
   spi_device_get_trans_result(self->spi, &rt, portMAX_DELAY);

   return data_read;
}

STATIC bool buffer_empty(stmpe610 *self) {
   uint8_t data;
   data = read_register_byte(self, STMPE_FIFO_STA)& STMPE_FIFO_STA_EMPTY;
   return (bool)data;
}

STATIC bool touched(stmpe610 *self) {
   uint8_t data;
   data = read_register_byte(self, STMPE_TSC_CTRL) & 0x80;
   return (bool)data;
}

STATIC mp_obj_t mp_stmpe610_init(mp_obj_t self_in) {
   esp_err_t ret;

   stmpe610_obj_t *self = MP_OBJ_TO_PTR(self_in);
   mp_activate_stmpe610(self_in);

   spi_device_interface_config_t devcfg = {
      .clock_speed_hz=self->mhz*1000*1000, //Clock out at DISP_SPI_MHZ MHz
      .mode=0,                             //SPI mode 0
      .spics_io_num=self->cs,                    //CS pin is set manually
      .queue_size=1,
      .pre_cb=NULL,
      .post_cb=NULL,
      .flags=SPI_DEVICE_HALFDUPLEX,
      .duty_cycle_pos=128,
   };

   gpio_pad_select_gpio(self->irq);
  
   gpio_set_direction(self->irq, GPIO_MODE_INPUT);
   
   //Attach the touch controller to the SPI bus
   ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi);
   if (ret != ESP_OK) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));
   }

   write_register(self, STMPE_SYS_CTRL2, 0x0); // turn on clocks!
   write_register(self, STMPE_TSC_CTRL,
		  STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
   // Serial.println(readRegister8(STMPE_TSC_CTRL), HEX);
   write_register(self, STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
   write_register(self, STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT |
		  (0x6 << 4)); // 96 clocks per conversion
   write_register(self, STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
   write_register(self, STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE |
		  STMPE_TSC_CFG_DELAY_1MS |
		  STMPE_TSC_CFG_SETTLE_5MS);
   write_register(self, STMPE_TSC_FRACTION_Z, 0x6);
   write_register(self, STMPE_FIFO_TH, 1);
   write_register(self, STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
   write_register(self, STMPE_FIFO_STA, 0); // unreset
   write_register(self, STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
   write_register(self, STMPE_INT_STA, 0xFF); // reset all ints
   write_register(self, STMPE_INT_CTRL,
		  STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);

   return mp_const_none;
}

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no more data to be read
 */
static bool stmpe610_read(lv_indev_data_t * data) {
   stmpe610_obj_t *self = MP_OBJ_TO_PTR(g_stmpe610 );
   if (!self || (!self->spi)) {
      nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "stmpe610 instance needs to be created before callback is called!"));
   }
   static int16_t last_x = 0;
   static int16_t last_y = 0;
   bool touched = touched(self);
   uint8_t buf;

   int16_t x = 0;
   int16_t y = 0;

   uint8_t read_data[4];
   
   if(touched == true) {
      for (uint8_t = 0; i < 4; i++) {
	 read_data = read_register_byte(STMPE_TSC_DATA_XYZ);
      }
      x = data[0];
      x <<= 4;
      x |= (data[1] >> 4);
      y = data[1] & 0x0F;
      y <<= 8;
      y |= data[2];
      last_x = x;
      last_y = y;
   } else {
      x = last_x;
      y = last_y;
   }

   data->point.x = x;
   data->point.y = y;
   data->state = touched == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

   return buffer_empty(self);
}
