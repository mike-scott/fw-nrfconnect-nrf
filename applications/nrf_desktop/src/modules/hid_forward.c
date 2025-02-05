/*
 * Copyright (c) 2018 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr/types.h>
#include <misc/slist.h>

#include <bluetooth/services/hids_c.h>
#include <misc/byteorder.h>

#define MODULE hid_forward
#include "module_state_event.h"

#include "hid_report_desc.h"
#include "config_channel.h"

#include "hid_event.h"
#include "ble_event.h"
#include "usb_event.h"
#include "config_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DESKTOP_BLE_SCANNING_LOG_LEVEL);

struct keyboard_event_item {
	sys_snode_t node;
	struct hid_keyboard_event *evt;
};

static struct bt_gatt_hids_c hidc[CONFIG_BT_MAX_CONN];
static const void *usb_id;
static atomic_t usb_ready;
static bool usb_busy;
static bool forward_pending;
static void *channel_id;

static struct hid_mouse_event *next_mouse_event;
static sys_slist_t keyboard_event_list;

static struct k_spinlock lock;

static void process_mouse_report(const u8_t *data)
{
	u8_t button_bm = data[0];
	s16_t wheel = (s8_t)data[1];

	u8_t x_buff[2] = {data[2], data[3] & 0x0F};
	u8_t y_buff[2] = {(data[3] >> 4) | ((data[4] << 4) & 0xF0), data[4] >> 4};

	u16_t x = sys_get_le16(x_buff);
	u16_t y = sys_get_le16(y_buff);

	if (x > REPORT_MOUSE_XY_MAX) {
		x |= 0xF000;
	}
	if (y > REPORT_MOUSE_XY_MAX) {
		y |= 0xF000;
	}

	struct hid_mouse_event *event;

	k_spinlock_key_t key = k_spin_lock(&lock);

	if (next_mouse_event) {
		__ASSERT_NO_MSG(usb_busy);

		LOG_WRN("Event override");
		event = next_mouse_event;
	} else {
		event = new_hid_mouse_event();
	}

	event->subscriber = usb_id;

	event->button_bm = button_bm;
	event->wheel     = wheel;

	event->dx = x;
	event->dy = y;

	if (!usb_busy) {
		EVENT_SUBMIT(event);
		usb_busy = true;
	} else {
		next_mouse_event = event;
	}

	k_spin_unlock(&lock, key);
}

static void process_keyboard_report(const u8_t *data)
{
	struct hid_keyboard_event *event = new_hid_keyboard_event();

	event->subscriber = usb_id;
	event->modifier_bm = data[0];
	memcpy(event->keys, &data[2], ARRAY_SIZE(event->keys));

	k_spinlock_key_t key = k_spin_lock(&lock);

	if (!usb_busy) {
		EVENT_SUBMIT(event);
		usb_busy = true;
	} else {
		struct keyboard_event_item *evt_item = k_malloc(sizeof(*evt_item));

		if (!evt_item) {
			LOG_ERR("OOM error");
			k_panic();
		}

		evt_item->evt = event;
		sys_slist_append(&keyboard_event_list, &evt_item->node);
	}
	k_spin_unlock(&lock, key);
}

static u8_t hidc_read(struct bt_gatt_hids_c *hids_c,
		      struct bt_gatt_hids_c_rep_info *rep,
		      u8_t err,
		      const u8_t *data)
{
	if (!data) {
		return BT_GATT_ITER_STOP;
	}

	if (err || !atomic_get(&usb_ready)) {
		return BT_GATT_ITER_CONTINUE;
	}

	switch (bt_gatt_hids_c_rep_id(rep)) {
	case REPORT_ID_MOUSE:
		process_mouse_report(data);
		break;

	case REPORT_ID_KEYBOARD_KEYS:
		process_keyboard_report(data);
		break;

	default:
		__ASSERT_NO_MSG(false);
	}

	return BT_GATT_ITER_CONTINUE;
}

static void hidc_ready(struct bt_gatt_hids_c *hids_c)
{
	struct bt_gatt_hids_c_rep_info *rep = NULL;

	while (NULL != (rep = bt_gatt_hids_c_rep_next(hids_c, rep))) {
		if (bt_gatt_hids_c_rep_type(rep) ==
		    BT_GATT_HIDS_C_REPORT_TYPE_INPUT) {
			int err = bt_gatt_hids_c_rep_subscribe(hids_c,
							       rep,
							       hidc_read);

			if (err) {
				LOG_ERR("Cannot subscribe to report (err:%d)",
					err);
			} else {
				LOG_INF("Subscriber to rep id:%d",
					bt_gatt_hids_c_rep_id(rep));
			}
			break;
		}
	}
}

static void hidc_pm_update(struct bt_gatt_hids_c *hids_c)
{
	LOG_INF("Protocol mode updated");
}

static void hidc_prep_error(struct bt_gatt_hids_c *hids_c, int err)
{
	if (err) {
		LOG_ERR("err:%d", err);
	}
}

static void init(void)
{
	static const struct bt_gatt_hids_c_init_params params = {
		.ready_cb = hidc_ready,
		.prep_error_cb = hidc_prep_error,
		.pm_update_cb = hidc_pm_update,
	};

	for (size_t i = 0; i < ARRAY_SIZE(hidc); i++) {
		bt_gatt_hids_c_init(&hidc[i], &params);
	}
	sys_slist_init(&keyboard_event_list);
}

static int assign_handles(struct bt_gatt_dm *dm)
{
	size_t i;
	for (i = 0; i < ARRAY_SIZE(hidc); i++) {
		if (!bt_gatt_hids_c_assign_check(&hidc[i])) {
			break;
		}
	}
	__ASSERT_NO_MSG(i < ARRAY_SIZE(hidc));

	int err = bt_gatt_hids_c_handles_assign(dm, &hidc[i]);

	if (err) {
		LOG_ERR("Cannot assign handles (err:%d)", err);
	}

	return err;
}

static void notify_config_forwarded(enum config_status status)
{
	struct config_forwarded_event *event = new_config_forwarded_event();

	forward_pending = (status == CONFIG_STATUS_PENDING);

	event->status = status;
	EVENT_SUBMIT(event);
}

static void hidc_write_cb(struct bt_gatt_hids_c *hidc,
			  struct bt_gatt_hids_c_rep_info *rep,
			  u8_t err)
{
	if (err) {
		LOG_WRN("Failed to write report: %d", err);
		notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
	} else {
		notify_config_forwarded(CONFIG_STATUS_SUCCESS);
	}
}

static u8_t hidc_read_cfg(struct bt_gatt_hids_c *hidc,
			  struct bt_gatt_hids_c_rep_info *rep,
			  u8_t err, const u8_t *data)
{
	if (err) {
		LOG_WRN("Failed to write report: %d", err);
		notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
	} else {
		struct config_channel_frame frame;

		int pos = config_channel_report_parse(data, REPORT_SIZE_USER_CONFIG,
						      &frame, false);
		if (pos < 0) {
			LOG_WRN("Could not set report");
			return pos;
		}

		if (frame.status != CONFIG_STATUS_SUCCESS) {
			LOG_INF("GATT read done, but fetch was not ready yet");

			/* Do not notify requester, host will schedule next read. */
			forward_pending = false;
			return 0;
		}

		struct config_fetch_event *event =
			new_config_fetch_event(frame.event_data_len);

		event->id = frame.event_id;
		event->recipient = frame.recipient;
		event->channel_id = channel_id;

		memcpy(event->dyndata.data, &data[pos], frame.event_data_len);

		EVENT_SUBMIT(event);
	}

	return 0;
}

static bool event_handler(const struct event_header *eh)
{
	if (is_hid_report_sent_event(eh)) {
		struct keyboard_event_item *next_item = NULL;
		k_spinlock_key_t key = k_spin_lock(&lock);
		sys_snode_t *kbd_event_node = sys_slist_get(&keyboard_event_list);

		if (kbd_event_node) {
			next_item = CONTAINER_OF(kbd_event_node,
						 struct keyboard_event_item,
						 node);

			EVENT_SUBMIT(next_item->evt);
		} else if (next_mouse_event) {
			EVENT_SUBMIT(next_mouse_event);
			next_mouse_event = NULL;
		} else {
			usb_busy = false;
		}
		k_spin_unlock(&lock, key);
		k_free(next_item);

		return false;
	}

	if (is_module_state_event(eh)) {
		const struct module_state_event *event =
			cast_module_state_event(eh);

		if (check_state(event, MODULE_ID(ble_state),
				MODULE_STATE_READY)) {
			static bool initialized;

			__ASSERT_NO_MSG(!initialized);
			initialized = true;

			init();
			module_set_state(MODULE_STATE_READY);
		}

		return false;
	}

	if (is_ble_discovery_complete_event(eh)) {
		const struct ble_discovery_complete_event *event =
			cast_ble_discovery_complete_event(eh);

		assign_handles(event->dm);
		return false;
	}

	if (is_hid_report_subscription_event(eh)) {
		const struct hid_report_subscription_event *event =
			cast_hid_report_subscription_event(eh);

		if (event->subscriber == usb_id) {
			atomic_set(&usb_ready, event->enabled);
		}

		return false;
	}

	if (is_ble_peer_event(eh)) {
		const struct ble_peer_event *event =
			cast_ble_peer_event(eh);

		if (event->state == PEER_STATE_DISCONNECTED) {
			for (size_t i = 0; i < ARRAY_SIZE(hidc); i++) {
				if ((bt_gatt_hids_c_assign_check(&hidc[i])) &&
				    (bt_gatt_hids_c_conn(&hidc[i]) == event->id)) {
					LOG_INF("HID device disconnected");
					bt_gatt_hids_c_release(&hidc[i]);
				}
			}
		}

		return false;
	}

	if (is_usb_state_event(eh)) {
		const struct usb_state_event *event =
			cast_usb_state_event(eh);

		switch (event->state) {
		case USB_STATE_POWERED:
			usb_id = event->id;
			break;
		case USB_STATE_DISCONNECTED:
			usb_id = NULL;
			atomic_set(&usb_ready, false);
			break;
		default:
			/* Ignore */
			break;
		}
		return false;
	}

	if (IS_ENABLED(CONFIG_DESKTOP_CONFIG_CHANNEL_ENABLE)) {
		/* TODO: Proper handling config event for multiple peers */
		if (is_config_forward_event(eh)) {
			const struct config_forward_event *event =
				cast_config_forward_event(eh);

			if (!bt_gatt_hids_c_ready_check(&hidc[0])) {
				LOG_WRN("Cannot forward, peer disconnected");

				notify_config_forwarded(CONFIG_STATUS_DISCONNECTED_ERROR);
				return false;
			}

			struct bt_gatt_hids_c_rep_info *config_rep =
				bt_gatt_hids_c_rep_find(&hidc[0],
					BT_GATT_HIDS_C_REPORT_TYPE_FEATURE,
					REPORT_ID_USER_CONFIG);
			if (!config_rep) {
				LOG_ERR("Feature report not found");
				notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
				return false;
			}

			if (event->dyndata.size > UCHAR_MAX) {
				LOG_ERR("Event data too big");
				__ASSERT_NO_MSG(false);
				return false;
			}

			struct config_channel_frame frame;
			u8_t report[REPORT_SIZE_USER_CONFIG];

			if (event->status == CONFIG_STATUS_FETCH) {
				LOG_INF("Forwarding fetch request");
				frame.status = CONFIG_STATUS_FETCH;
			}

			frame.recipient = event->recipient;
			frame.event_id = event->id;
			frame.event_data_len = event->dyndata.size;
			frame.event_data = (u8_t *) event->dyndata.data;

			int pos = config_channel_report_fill(report, sizeof(report), &frame, false);
			if (pos < 0) {
				LOG_WRN("Could not set report");
				return pos;
			}

			int err = bt_gatt_hids_c_rep_write(&hidc[0], config_rep,
					hidc_write_cb, report, sizeof(report));
			if (err) {
				LOG_ERR("Writing report failed, err:%d", err);
				notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
			}

			return false;
		}

		if (is_config_forward_get_event(eh)) {
			const struct config_forward_get_event *event =
				cast_config_forward_get_event(eh);

			if (forward_pending) {
				LOG_DBG("GATT read already pending");
				return false;
			} else {
				if (!bt_gatt_hids_c_ready_check(&hidc[0])) {
					LOG_WRN("Cannot forward, peer disconnected");

					notify_config_forwarded(CONFIG_STATUS_DISCONNECTED_ERROR);
					return false;
				}

				struct bt_gatt_hids_c_rep_info *config_rep =
					bt_gatt_hids_c_rep_find(&hidc[0],
						BT_GATT_HIDS_C_REPORT_TYPE_FEATURE,
						REPORT_ID_USER_CONFIG);
				if (!config_rep) {
					LOG_ERR("Feature report not found");
					notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
					return false;
				}

				notify_config_forwarded(CONFIG_STATUS_PENDING);

				channel_id = event->channel_id;

				int err = bt_gatt_hids_c_rep_read(&hidc[0], config_rep,
								  hidc_read_cfg);
				if (err) {
					LOG_ERR("Reading report failed, err:%d", err);
					notify_config_forwarded(CONFIG_STATUS_WRITE_ERROR);
				}

				return false;
			}
		}
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}
EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, ble_discovery_complete_event);
EVENT_SUBSCRIBE(MODULE, ble_peer_event);
EVENT_SUBSCRIBE(MODULE, usb_state_event);
EVENT_SUBSCRIBE(MODULE, hid_report_subscription_event);
EVENT_SUBSCRIBE(MODULE, hid_report_sent_event);
#if CONFIG_DESKTOP_CONFIG_CHANNEL_ENABLE
EVENT_SUBSCRIBE(MODULE, config_forward_event);
EVENT_SUBSCRIBE(MODULE, config_forward_get_event);
#endif
