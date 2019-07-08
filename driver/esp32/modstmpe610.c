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

//////////////////////////////////////////////////////////////////////////////
// Module definition
//////////////////////////////////////////////////////////////////////////////

typedef struct _stmpe610_obj_t
{
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

STATIC mp_obj_t mp_activate_stmpe610(mp_obj_t self_in)
{
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
                               const mp_obj_t *all_args)
{
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

STATIC mp_obj_t mp_stmpe610_init(mp_obj_t self_in)
{
    esp_err_t ret;

    stmpe610_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_activate_stmpe610(self_in);

    spi_device_interface_config_t devcfg={
		.clock_speed_hz=self->mhz*1000*1000, //Clock out at DISP_SPI_MHZ MHz
		.mode=0,                             //SPI mode 0
		.spics_io_num=-1,                    //CS pin is set manually
		.queue_size=1,
		.pre_cb=NULL,
		.post_cb=NULL,
		.flags=SPI_DEVICE_HALFDUPLEX,
		.duty_cycle_pos=128,
	};


    gpio_set_direction(self->irq, GPIO_MODE_INPUT);
    gpio_set_direction(self->cs, GPIO_MODE_OUTPUT);
    gpio_set_level(self->cs, 1);

    //Attach the touch controller to the SPI bus
    ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi);
    if (ret != ESP_OK) nlr_raise(
        mp_obj_new_exception_msg(&mp_type_RuntimeError, "Failed adding SPI device"));

    return mp_const_none;
}

STATIC mp_obj_t mp_stmpe610_deinit(mp_obj_t self_in)
{
    //stmpe610_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return mp_const_none;
}

static void stmpe610_corr(stmpe610_obj_t *self, int16_t * x, int16_t * y);
static void stmpe610_avg(stmpe610_obj_t *self, int16_t * x, int16_t * y);
static uint8_t tp_spi_xchg(stmpe610_obj_t *self, uint8_t data_send);

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
static bool stmpe610_read(lv_indev_data_t * data)
{
    stmpe610_obj_t *self = MP_OBJ_TO_PTR(g_stmpe610 );
    if (!self || (!self->spi)) nlr_raise(
            mp_obj_new_exception_msg(
                &mp_type_RuntimeError, "stmpe610 instance needs to be created before callback is called!"));
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    bool valid = true;
    uint8_t buf;

    int16_t x = 0;
    int16_t y = 0;

    uint8_t irq = gpio_get_level(self->irq);

    if(irq == 0) {
        gpio_set_level(self->cs, 0);
        tp_spi_xchg(self, CMD_X_READ);         /*Start x read*/

        buf = tp_spi_xchg(self, 0);           /*Read x MSB*/
        x = buf << 8;
        buf = tp_spi_xchg(self, CMD_Y_READ);  /*Until x LSB converted y command can be sent*/
        x += buf;

        buf =  tp_spi_xchg(self, 0);   /*Read y MSB*/
        y = buf << 8;

        buf =  tp_spi_xchg(self, 0);   /*Read y LSB*/
        y += buf;
        gpio_set_level(self->cs, 1);

        /*Normalize Data*/
        x = x >> 3;
        y = y >> 3;
        stmpe610_corr(self, &x, &y);
        stmpe610_avg(self, &x, &y);
        last_x = x;
        last_y = y;


    } else {
        x = last_x;
        y = last_y;
        self->avg_last = 0;
        valid = false;
    }

    data->point.x = x;
    data->point.y = y;
    data->state = valid == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

    return valid;
}

/**********************
 *   HELPER FUNCTIONS
 **********************/

static uint8_t tp_spi_xchg(stmpe610_obj_t *self, uint8_t data_send)
{
    uint8_t data_rec = 0;
	spi_transaction_t t;
    memset(&t, 0, sizeof(t));  //Zero out the transaction
	t.length = 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = &data_send;  //Data
	t.rx_buffer = &data_rec;

	spi_device_queue_trans(self->spi, &t, portMAX_DELAY);

	spi_transaction_t * rt;
	spi_device_get_trans_result(self->spi, &rt, portMAX_DELAY);

	return data_rec;
}
