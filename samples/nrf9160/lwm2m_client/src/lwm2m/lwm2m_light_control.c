/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define LOG_MODULE_NAME app_lwm2m_light
#define LOG_LEVEL CONFIG_APP_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <dk_buttons_and_leds.h>
#include <net/lwm2m.h>

#define LIGHT_NAME	"LED1"

static u32_t led_state;

/* TODO: Move to a pre write hook that can handle ret codes once available */
static int lc_on_off_cb(u16_t obj_inst_id, u16_t res_id, u16_t res_inst_id,
			u8_t *data, u16_t data_len,
			bool last_block, size_t total_size)
{
	int ret = 0;
	u32_t led_val;

	led_val = *(u8_t *) data;
	if (led_val != led_state) {
		ret = dk_set_led(DK_LED1, led_val);
		if (ret) {
			/*
			 * We need an extra hook in LWM2M to better handle
			 * failures before writing the data value and not in
			 * post_write_cb, as there is not much that can be
			 * done here.
			 */
			LOG_ERR("Fail to turn on LED1");
			return ret;
		}

		led_state = led_val;
		/* TODO: Move to be set by an internal post write function */
		lwm2m_engine_set_s32("3311/0/5852", 0);
	}

	return ret;
}

int lwm2m_init_light_control(void)
{
	int ret;

	/* start with LED off */
	ret = dk_set_led(DK_LED1, 0);
	if (ret) {
		return ret;
	}

	/* create light control device */
	lwm2m_engine_create_obj_inst("3311/0");
	lwm2m_engine_register_post_write_callback("3311/0/5850", lc_on_off_cb);
	lwm2m_engine_set_res_data("3311/0/5750",
				  LIGHT_NAME, sizeof(LIGHT_NAME),
				  LWM2M_RES_DATA_FLAG_RO);

	return 0;
}
