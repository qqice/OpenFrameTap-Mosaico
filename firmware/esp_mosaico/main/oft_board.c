/* GPIO/driver references: espressif/esp_boards 0.6.1 esp_mosaico V1.0.
 * No private panel init sequence: use the CO5300 driver's stock QSPI sequence. */
#include "oft_board.h"
#include "oft_ui.h"
#include "oft_touch.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_touch_cst9217.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include <stdatomic.h>

static const char *TAG="oft_board";
static atomic_uint_least32_t flushes,unaligned_flushes;
static atomic_uint_least32_t render_last_us,render_max_us;
static int64_t render_start_us;
static void observe_render(lv_event_t *event)
{
    if(lv_event_get_code(event)==LV_EVENT_REFR_START)render_start_us=esp_timer_get_time();
    else if(render_start_us){
        uint32_t elapsed=(uint32_t)(esp_timer_get_time()-render_start_us);
        atomic_store(&render_last_us,elapsed);
        if(elapsed>atomic_load(&render_max_us))atomic_store(&render_max_us,elapsed);
    }
}
static bool alignment_selftest_passed;
static i2c_master_bus_handle_t shared_i2c;
i2c_master_bus_handle_t oft_board_i2c(void){return shared_i2c;}

static void co5300_round_area(lv_area_t *area)
{
    /* Inclusive LVGL rectangle: even starts and odd ends give even dimensions.
       Run BEFORE rendering, not by moving the window after pixels are drawn.
       LVGL 9.5 get_max_row() also invokes this hook when splitting the small
       draw buffer, so each partial strip retains even height. */
    area->x1 &= ~1;
    area->y1 &= ~1;
    area->x2 |= 1;
    area->y2 |= 1;
}
static bool valid_window(const lv_area_t *a)
{
    return a->x1>=0 && a->y1>=0 && a->x2<480 && a->y2<480 &&
        a->x1<=a->x2 && a->y1<=a->y2 &&
        !(a->x1&1) && !(a->y1&1) && (a->x2&1) && (a->y2&1);
}
static bool alignment_selftest(void)
{
    const int starts[]={0,1,2,105,239,478,479};
    const int lengths[]={1,2,3,27,54,260,480};
    for(unsigned i=0;i<sizeof(starts)/sizeof(starts[0]);i++)
        for(unsigned j=0;j<sizeof(lengths)/sizeof(lengths[0]);j++) {
            int lo=starts[i],hi=lo+lengths[j]-1;
            if(hi>=480)hi=479;
            lv_area_t a={lo,lo,hi,hi};
            co5300_round_area(&a);
            if(!valid_window(&a)||a.x1>lo||a.y1>lo||a.x2<hi||a.y2<hi)return false;
            lv_area_t unchanged=a;
            co5300_round_area(&a);
            if(a.x1!=unchanged.x1||a.x2!=unchanged.x2||a.y1!=unchanged.y1||a.y2!=unchanged.y2)return false;
        }
    /* A 174-pixel-wide invalidation yields 27 rows in 4800 pixels. Round the
       chunk down to 26, rather than letting the last row overrun the buffer. */
    int max_rows=4800/174;
    lv_area_t chunk={0,0,0,max_rows-1};
    co5300_round_area(&chunk);
    if(chunk.y2+1>max_rows){chunk.y2=max_rows-2;co5300_round_area(&chunk);}
    return chunk.y2+1==26 && 174*(chunk.y2+1)<=4800;
}
static void observe_flush(lv_event_t *event)
{
    const lv_area_t *area=lv_event_get_param(event);
    atomic_fetch_add_explicit(&flushes,1,memory_order_relaxed);
    if(!area||!valid_window(area))atomic_fetch_add_explicit(&unaligned_flushes,1,memory_order_relaxed);
}
oft_display_stats_t oft_board_display_stats(void)
{
    return (oft_display_stats_t){
        .flushes=atomic_load_explicit(&flushes,memory_order_relaxed),
        .unaligned_flushes=atomic_load_explicit(&unaligned_flushes,memory_order_relaxed),
        .render_last_us=atomic_load(&render_last_us),.render_max_us=atomic_load(&render_max_us),
        .alignment_selftest_passed=alignment_selftest_passed};
}
esp_err_t oft_board_display_start(void)
{
    alignment_selftest_passed=alignment_selftest();
    ESP_RETURN_ON_FALSE(alignment_selftest_passed,ESP_FAIL,TAG,"alignment self-test");
    /* Mosaico peripheral rail: active low. Audio remains off; do not drive
       GPIO57 (whole-device power-off), boot strap or NAND/expansion pins. */
    gpio_config_t power={.pin_bit_mask=1ULL<<60,.mode=GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&power),TAG,"power GPIO");
    gpio_set_level(60,0);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_master_bus_handle_t bus=NULL;
    i2c_master_bus_config_t i2c={.i2c_port=0,.sda_io_num=0,.scl_io_num=1,
        .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,.flags.enable_internal_pullup=true};
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c,&bus),TAG,"I2C bus");
    shared_i2c=bus;
    spi_bus_config_t spi=CO5300_PANEL_BUS_QSPI_CONFIG(44,36,51,35,9,480*40*2);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST,&spi,SPI_DMA_CH_AUTO),TAG,"QSPI bus");
    esp_lcd_panel_io_handle_t io=NULL;
    esp_lcd_panel_io_spi_config_t io_cfg=CO5300_PANEL_IO_QSPI_CONFIG(50,NULL,NULL);
    io_cfg.flags.psram_dma_direct=true;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,&io_cfg,&io),TAG,"panel IO");
    co5300_vendor_config_t vendor={.flags.use_qspi_interface=1};
    const esp_lcd_panel_dev_config_t panel_cfg={.reset_gpio_num=42,
        .rgb_ele_order=LCD_RGB_ELEMENT_ORDER_RGB,.bits_per_pixel=16,.vendor_config=&vendor};
    esp_lcd_panel_handle_t panel=NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(io,&panel_cfg,&panel),TAG,"CO5300");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel),TAG,"panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel),TAG,"panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel,true),TAG,"panel enable");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_co5300_set_brightness(panel,50),TAG,"panel brightness");

    lvgl_port_cfg_t lv_cfg=ESP_LVGL_PORT_INIT_CONFIG();
    lv_cfg.task_priority=3;lv_cfg.task_affinity=0;
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lv_cfg),TAG,"LVGL");
    const lvgl_port_display_cfg_t display_cfg={.io_handle=io,.panel_handle=panel,
        .buffer_size=480*40,.double_buffer=true,.hres=480,.vres=480,.monochrome=false,
        .color_format=LV_COLOR_FORMAT_RGB565,
        .rounder_cb=co5300_round_area,
        .rotation={.swap_xy=false,.mirror_x=false,.mirror_y=false},
        .flags={.buff_dma=true,.buff_spiram=true,.swap_bytes=true}};
    lv_display_t *display=lvgl_port_add_disp(&display_cfg);
    ESP_RETURN_ON_FALSE(display,ESP_FAIL,TAG,"LVGL display");
    lv_display_add_event_cb(display,observe_flush,LV_EVENT_FLUSH_START,NULL);
    lv_display_add_event_cb(display,observe_render,LV_EVENT_REFR_START,NULL);
    lv_display_add_event_cb(display,observe_render,LV_EVENT_REFR_READY,NULL);
    esp_lcd_panel_io_handle_t touch_io=NULL;
    esp_lcd_panel_io_i2c_config_t touch_io_cfg=ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
    touch_io_cfg.scl_speed_hz=400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus,&touch_io_cfg,&touch_io),TAG,"touch IO");
    const esp_lcd_touch_config_t touch_cfg={.x_max=480,.y_max=480,
        .rst_gpio_num=-1,.int_gpio_num=6,
        .levels={.reset=0,.interrupt=0},
        .flags={.swap_xy=false,.mirror_x=false,.mirror_y=false}};
    esp_lcd_touch_handle_t touch=NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_cst9217(touch_io,&touch_cfg,&touch),TAG,"touch probe");
    if (lvgl_port_lock(1000)) {
        esp_err_t input_result=oft_touch_start(touch,display);
        if(input_result==ESP_OK)oft_ui_create();
        lvgl_port_unlock();
        ESP_RETURN_ON_ERROR(input_result,TAG,"Independent touch sampler");
    }
    else return ESP_ERR_TIMEOUT;
    ESP_LOGI(TAG,"display=480x480 CO5300; touch=CST9220; geometry acceptance pending");
    return ESP_OK;
}
