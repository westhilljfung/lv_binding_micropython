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
      .miso_io_num=GPIO_NUM_19,
      .mosi_io_num=GPIO_NUM_18,
      .sclk_io_num=GPIO_NUM_5,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
   };

   gpio_pad_select_gpio(GPIO_NUM_19);
   gpio_pad_select_gpio(GPIO_NUM_18);
   gpio_pad_select_gpio(GPIO_NUM_5);

   gpio_set_level(GPIO_NUM_19, 1);
   gpio_set_level(GPIO_NUM_18, 1);
   gpio_set_level(GPIO_NUM_5, 1);
	
   gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
   gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);
   gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

   gpio_pad_select_gpio(GPIO_NUM_14);
   gpio_pad_select_gpio(GPIO_NUM_15);
   gpio_pad_select_gpio(GPIO_NUM_32);

   gpio_set_level(GPIO_NUM_14, 1);
   gpio_set_level(GPIO_NUM_15, 1);
   gpio_set_level(GPIO_NUM_32, 1);
   
   gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
   gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
   
   gpio_set_level(GPIO_NUM_14, 1);
   gpio_set_level(GPIO_NUM_15, 1);
   gpio_set_level(GPIO_NUM_32, 1);
   
   ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
   ESP_ERROR_CHECK(ret);

   spi_device_handle_t spi;
   //Attach the Touch Screen to the SPI bus
   spi_device_interface_config_t devcfg={
      .clock_speed_hz=500000, //Clock out at 1 MHz
      .mode=0,                             //SPI mode 0
      .spics_io_num=-1,              //CS pin
      .queue_size=1,
      .command_bits=8,
   };

   ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
   ESP_ERROR_CHECK(ret);

   gpio_set_level(GPIO_NUM_32, 0);   
   vTaskDelay(100 / portTICK_RATE_MS);
   
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

   ret = spi_device_transmit(spi, &t);

   ESP_ERROR_CHECK(ret);
   
   printf("Read Data: %x %x %x %x\n", read_data[0], read_data[1], read_data[2], read_data[3]);

   return mp_const_none;
}
