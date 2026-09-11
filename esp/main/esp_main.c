/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_psram.h"
#include "esp_partition.h"
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "../../ini.h"
#include "../../pc.h"
#include "common.h"
#include "../../i8042.h"
#include "driver/adc.h"
#include "driver/i2c.h"

//
#include "esp_private/system_internal.h"
uint32_t get_uticks()
{
	return esp_system_get_time();
}

void *psmalloc(long size);
void *fbmalloc(long size);
void *bigmalloc(size_t size)
{
	return psmalloc(size);
}

static char *pcram;
static long pcram_off;
static long pcram_len;
void *pcmalloc(long size)
{
	void *ret = pcram + pcram_off;

	size = (size + 31) / 32 * 32;
	if (pcram_off + size > pcram_len) {
		fprintf(stderr, "pcram error %ld %ld %ld\n", size, pcram_off, pcram_len);
		abort();
	}
	pcram_off += size;
	return ret;
}

void pcmalloc_init(void *ptr, long len)
{
	pcram = ptr;
	pcram_len = len;
}

int load_rom(void *phys_mem, const char *file, uword addr, int backward)
{
	if (file && file[0] == '/') {
		FILE *fp = fopen(file, "rb");
		assert(fp);
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fprintf(stderr, "%s len %d\n", file, len);
		rewind(fp);
		if (backward)
			fread(phys_mem + addr - len, 1, len, fp);
		else
			fread(phys_mem + addr, 1, len, fp);
		fclose(fp);
		return len;
	}
	const esp_partition_t *part =
		esp_partition_find_first(ESP_PARTITION_TYPE_ANY,
					 ESP_PARTITION_SUBTYPE_ANY,
					 file);
	assert(part);
	int len = part->size;
	fprintf(stderr, "%s len %d\n", file, len);
	if (backward)
		esp_partition_read(part, 0, phys_mem + addr - len, len);
	else
		esp_partition_read(part, 0, phys_mem + addr, len);
	return len;
}

//
EventGroupHandle_t global_event_group;
struct Globals globals;

typedef struct {
	PC *pc;
	u8 *fb1;
	u8 *fb;
} Console;

#define NN 32
Console *console_init(int width, int height)
{
	Console *c = malloc(sizeof(Console));
	c->fb1 = fbmalloc(LCD_WIDTH * LCD_HEIGHT / NN * 2);
	if (globals.panel_fb) {
		/* Zero-copy: VGA renders directly into the RGB DMA frame buffer */
		c->fb = globals.panel_fb;
	} else {
		c->fb = bigmalloc(LCD_WIDTH * LCD_HEIGHT * 2);
	}
	return c;
}

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src);
static void redraw(void *opaque,
		   int x, int y, int w, int h)
{
	Console *s = opaque;
	for (int i = 0; i < NN; i++) {
		uint16_t *src = (uint16_t *) s->fb;
		src += LCD_WIDTH * LCD_HEIGHT / NN * i;
		memcpy(s->fb1, src, LCD_WIDTH * LCD_HEIGHT / NN * 2);
		lcd_draw(0, LCD_WIDTH / NN * i,
			 LCD_HEIGHT, LCD_WIDTH / NN * (i + 1),
			 s->fb1);
		vga_step(s->pc->vga);
		usleep(900);
	}
}

static int pc_main(const char *file)
{
	PCConfig conf;
	memset(&conf, 0, sizeof(conf));
	conf.mem_size = 8 * 1024 * 1024;
	conf.vga_mem_size = 256 * 1024;
	conf.cpu_gen = 4;
	conf.fpu = 0;

	int err = ini_parse(file, parse_conf_ini, &conf);
	if (err) {
		fprintf(stderr, "error %d\n", err);
		return err;
	}

	if (conf.width != LCD_WIDTH || conf.height != LCD_HEIGHT) {
		fprintf(stderr, "fixing width/height mismatch %dx%d => %dx%d\n",
			conf.width, conf.height, LCD_WIDTH, LCD_HEIGHT);
		conf.width = LCD_WIDTH;
		conf.height = LCD_HEIGHT;
	}

	Console *console = console_init(conf.width, conf.height);
	PC *pc = pc_new(redraw, console, console->fb, &conf);
	console->pc = pc;
	globals.pc = pc;
	globals.kbd = pc->kbd;
	globals.mouse = pc->mouse;
	xEventGroupSetBits(global_event_group, BIT0);

	load_bios_and_reset(pc);

	pc->boot_start_time = get_uticks();
	for (; pc->shutdown_state != 8;) {
		pc_step(pc);
	}
	return 0;
}

// НАСТРОЙКА МЫШКИ, ДЖОЙСТИКА И КЛАВИАТУРЫ
#define I2C_MASTER_NUM              I2C_NUM_0  
#define CARDKB_I2C_ADDR             0x5F       

#define PIN_BUTTON_LEFT             10  // Левая кнопка мыши (клик)
#define PIN_BUTTON_RIGHT            11  // Правая кнопка мыши (клик)
#define PIN_JOYSTICK_X              4   // Ось X джойстика
#define PIN_JOYSTICK_Y              5   // Ось Y джойстика

void cardkb_task(void *pvParameters) {
    uint8_t asc_code;
    uint8_t last_scancode = 0;

    // 1. Настройка кнопок клика
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_LEFT) | (1ULL << PIN_BUTTON_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULL_UP_ENABLE,
        .pull_down_en = GPIO_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 2. Настройка аналоговых входов для джойстика
    gpio_set_direction(PIN_JOYSTICK_X, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_JOYSTICK_Y, GPIO_MODE_INPUT);

    while (1) {
        // --- 1. ОБРАБОТКА МЫШКИ И ДЖОЙСТИКА ---
        if (globals.mouse) {
            bool left_click = (gpio_get_level(PIN_BUTTON_LEFT) == 0);
            bool right_click = (gpio_get_level(PIN_BUTTON_RIGHT) == 0);

            globals.mouse->button_state = (left_click ? 1 : 0) | (right_click ? 2 : 0);
            
            int raw_x = adc1_get_raw(ADC1_CHANNEL_4); 
            int raw_y = adc1_get_raw(ADC1_CHANNEL_5); 

            int dx = 0;
            int dy = 0;
            
            if (raw_x > 2200 || raw_x < 1900) dx = (raw_x - 2048) / 150;
            if (raw_y > 2200 || raw_y < 1900) dy = (raw_y - 2048) / 150;

            if (dx != 0 || dy != 0) {
                mouse_move_relative(dx, dy); 
            }
        }

        // --- 2. ОБРАБОТКА КЛАВИАТУРЫ CARDKB ---
        if (last_scancode != 0) {
            i8042_write_data(last_scancode | 0x80); 
            last_scancode = 0;
            vTaskDelay(pdMS_TO_TICKS(20)); 
        }

        esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, CARDKB_I2C_ADDR, &asc_code, 1, pdMS_TO_TICKS(30));
        
        if (ret == ESP_OK && asc_code != 0) {
            uint8_t scancode = 0;
            if (asc_code >= 'a' && asc_code <= 'z') {
                static const uint8_t map[] = { 0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, 0x17, 0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19, 0x10, 0x13, 0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, 0x15, 0x2C };
                scancode = map[asc_code - 'a'];
            } else if (asc_code >= '0' && asc_code <= '9') {
                static const uint8_t map_num[] = { 0x0B, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A };
                scancode = map_num[asc_code - '0'];
            } else if (asc_code == 0x0D) { scancode = 0x1C; } 
            else if (asc_code == 0x08) { scancode = 0x0E; } 
            else if (asc_code == 0x20) { scancode = 0x39; } 

            if (scancode != 0) {
                i8042_write_data(scancode); 
                last_scancode = scancode;   
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40)); 
    }
}

void *esp_psram_get(size_t *size);
void vga_task(void *arg);
void i2s_main();
void wifi_main(const char *, const char *);
void storage_init(void);
void usb_setup(void);

struct esp_ini_config {
	const char *filename;
	char ssid[16];
	char pass[32];
	int enable_usb;
};

static void i386_task(void *arg)
{
	struct esp_ini_config *config = arg;
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "main runs on core %d\n", core_id);
	xEventGroupWaitBits(global_event_group,
	                    BIT1,
	                    pdFALSE,
	                    pdFALSE,
	                    portMAX_DELAY);
	pc_main(config->filename);
	vTaskDelete(NULL);
}

static char *psram;
static long psram_off;
static long psram_len;
void *psmalloc(long size)
{
	void *ret = psram + psram_off;

	size = (size + 4095) / 4096 * 4096;
	if (psram_off + size > psram_len) {
		fprintf(stderr, "psram error %ld %ld %ld\n", size, psram_off, psram_len);
		abort();
	}
	psram_off += size;
	return ret;
}

void *fbmalloc(long size)
{
	void *fb = (uint8_t *) heap_caps_calloc(1, size, MALLOC_CAP_DMA);
	if (!fb) {
		fprintf(stderr, "fbmalloc error %ld\n", size);
		abort();
	}
	return fb;
}

static int parse_ini(void* user, const char* section,
		     const char[name], const char* value)
{
	struct esp_ini_config *conf = user;
#define SEC(a) (strcmp(section, a) == 0)
#define NAME(a) (strcmp(name, a) == 0)
	if (SEC("esp")) {
		if (NAME("ssid")) {
			if (strlen(value) < 32)
				strcpy(conf->ssid, value);
		} else if (NAME("pass")) {
			if (strlen(value) < 64)
				strcpy(conf->pass, value);
		} else if (NAME("enable_usb")) {
			conf->enable_usb = atoi(value);
		}
	}
#undef SEC
#undef NAME
	return 1;
}

void app_main(void)
{
	global_event_group = xEventGroupCreate();

#ifdef ESPDEBUG
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity	= UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	uart_param_config(UART_NUM_0, &uart_config);
	if (uart_driver_install(UART_NUM_0, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
		assert(false);
	}
#endif

	i2s_main();
	storage_init();

	esp_psram_init();
#ifndef PSRAM_ALLOC_LEN
	size_t len;
	psram = esp_psram_get(&len);
	psram_len = len;
#define PSRAM_ALLOC_LEN
	psram_len = PSRAM_ALLOC_LEN;
	psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
#endif

	const static char *files[] = {
		"/sdcard/tiny386.ini",
		"/spiflash/tiny386.ini",
		NULL,
	};
	static struct esp_ini_config config;
	for (int i = 0; files[i]; i++) {
		if (ini_parse(files[i], parse_ini, &config) == 0) {
			config.filename = files[i];
			break;
		}
	}

	if (config.enable_usb) {
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		usb_setup();
	}

	if (config.ssid[0]) {
		wifi_main(config.ssid, config.pass);
	}

	xTaskCreatePinnedToCore(cardkb_task, "cardkb_task", 3072, NULL, 5, NULL, 1);
    
	if (psram) {
		xTaskCreatePinnedToCore(i386_task, "i386_main", 4096, &config, 3, NULL, 1);
		xTaskCreatePinnedToCore(vga_task, "vga_task", 4096, NULL, 0, NULL, 0);
	}
}
