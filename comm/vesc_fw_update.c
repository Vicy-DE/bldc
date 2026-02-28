/*
	Copyright 2025 RC-Plane-Project Contributors

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/**
 * @file vesc_fw_update.c
 * @brief Remote VESC firmware update over CAN — implementation.
 *
 * Sends COMM_ERASE_NEW_APP / COMM_WRITE_NEW_APP_DATA /
 * COMM_JUMP_TO_BOOTLOADER to a target VESC over CAN and handles the
 * acknowledgment responses using a response-capture hook.
 *
 * The module registers a custom send function with commands_process_packet
 * so that when the target VESC's response arrives back over CAN, it is
 * captured into a local buffer and a ChibiOS binary semaphore is signaled.
 *
 * Protocol reference: commands.c COMM_ERASE_NEW_APP / COMM_WRITE_NEW_APP_DATA.
 */

#include "vesc_fw_update.h"
#include "comm_can.h"
#include "commands.h"
#include "buffer.h"
#include "terminal.h"
#include "datatypes.h"
#include "ch.h"
#include "hal.h"

#include <string.h>

/* ── Configuration ────────────────────────────────────────────────── */

/** Maximum firmware chunk size (bytes of firmware data per write command).
 *  1 byte COMM_ID + 4 bytes offset + chunk = VESC RX buffer (512).
 *  Use 500 to stay safely within limits. */
#define FW_UPDATE_CHUNK_SIZE_DEFAULT    500

/** Erase timeout default (ms). Flash erase takes several seconds. */
#define FW_UPDATE_ERASE_TIMEOUT_DEFAULT 15000

/** Per-chunk write timeout default (ms). */
#define FW_UPDATE_WRITE_TIMEOUT_DEFAULT 5000

/* ── Response capture state ───────────────────────────────────────── */

static volatile bool     fw_resp_received;
static uint8_t           fw_resp_buf[16];
static volatile unsigned fw_resp_len;
static thread_t         *fw_waiting_tp;

/**
 * @brief Response capture callback for firmware update commands.
 *
 * Called by commands_process_packet when a CAN response from the target
 * VESC arrives.  Copies the response data and signals the waiting thread.
 *
 * @param[in] data  Response packet data.
 * @param[in] len   Response data length.
 *
 * @sideeffects Writes to fw_resp_buf, signals fw_waiting_tp.
 */
static void fw_update_reply_capture(unsigned char *data, unsigned int len) {
	if (len > sizeof(fw_resp_buf)) {
		len = sizeof(fw_resp_buf);
	}
	memcpy(fw_resp_buf, data, len);
	fw_resp_len = len;
	fw_resp_received = true;

	if (fw_waiting_tp) {
		chEvtSignal(fw_waiting_tp, (eventmask_t)1);
	}
}

/**
 * @brief Wait for a firmware update response with timeout.
 *
 * @param[in] timeout_ms  Maximum wait time in milliseconds.
 * @return true if response received, false on timeout.
 *
 * @sideeffects Blocks the calling thread for up to timeout_ms.
 */
static bool fw_update_wait_response(uint32_t timeout_ms) {
	fw_waiting_tp = chThdGetSelfX();
	chEvtGetAndClearEvents(ALL_EVENTS);

	systime_t start = chVTGetSystemTimeX();
	systime_t timeout_ticks = MS2ST(timeout_ms);

	while (!fw_resp_received) {
		systime_t elapsed = chVTTimeElapsedSinceX(start);
		if (elapsed >= timeout_ticks) {
			fw_waiting_tp = NULL;
			return false;
		}
		chEvtWaitAnyTimeout((eventmask_t)1, timeout_ticks - elapsed);
	}

	fw_waiting_tp = NULL;
	return true;
}

/* ── Public API ───────────────────────────────────────────────────── */

/* See vesc_fw_update.h for full documentation. */
fw_update_result_t vesc_fw_update_erase(uint8_t controller_id,
                                        uint32_t fw_size,
                                        uint32_t timeout_ms)
{
	if (fw_size == 0) {
		return FW_UPDATE_ERR_PARAM;
	}

	/* Build COMM_ERASE_NEW_APP payload: [COMM_ID][uint32 fw_size] */
	uint8_t payload[5];
	int32_t ind = 0;
	payload[ind++] = COMM_ERASE_NEW_APP;
	buffer_append_uint32(payload, fw_size, &ind);

	/* Reset response state */
	fw_resp_received = false;
	fw_resp_len = 0;

	/* Send via CAN transport with send=0 (request response).
	 * The target VESC will process the command and send back a
	 * response containing [COMM_ERASE_NEW_APP, success_byte].
	 *
	 * When send=0, the target calls reply_func which sends back
	 * to us via send_packet_wrapper → comm_can_send_buffer(our_id, ..., 1).
	 * On our side, send=1 dispatches to commands_send_packet_can_last.
	 *
	 * We temporarily set send_func_can_fwd to our capture callback
	 * by sending a command with our capture as the reply_func.    */
	comm_can_send_buffer(controller_id, payload, (unsigned int)ind, 0);

	/* Wait for erase acknowledgment */
	if (!fw_update_wait_response(timeout_ms)) {
		return FW_UPDATE_ERR_TIMEOUT;
	}

	/* Parse response: [COMM_ERASE_NEW_APP][success_byte] */
	if (fw_resp_len < 2) {
		return FW_UPDATE_ERR_ERASE;
	}
	if (fw_resp_buf[0] != COMM_ERASE_NEW_APP) {
		return FW_UPDATE_ERR_ERASE;
	}
	if (fw_resp_buf[1] != 1) {
		return FW_UPDATE_ERR_ERASE;
	}

	return FW_UPDATE_OK;
}

/* See vesc_fw_update.h for full documentation. */
fw_update_result_t vesc_fw_update_write(uint8_t controller_id,
                                        const uint8_t *fw_data,
                                        uint32_t fw_size,
                                        uint32_t chunk_size,
                                        uint32_t timeout_ms,
                                        fw_update_progress_cb_t progress_cb)
{
	if (fw_data == NULL || fw_size == 0) {
		return FW_UPDATE_ERR_PARAM;
	}

	if (chunk_size == 0 || chunk_size > FW_UPDATE_CHUNK_SIZE_DEFAULT) {
		chunk_size = FW_UPDATE_CHUNK_SIZE_DEFAULT;
	}

	/* Allocate payload buffer on stack:
	 * [COMM_WRITE_NEW_APP_DATA][uint32 offset][firmware_data...]
	 * Total: 505 bytes — requires a thread stack >= 1024 bytes. */
	uint8_t payload[1 + 4 + FW_UPDATE_CHUNK_SIZE_DEFAULT];

	uint32_t offset = 0;
	while (offset < fw_size) {
		uint32_t chunk = chunk_size;
		if (offset + chunk > fw_size) {
			chunk = fw_size - offset;
		}

		/* Build payload */
		int32_t ind = 0;
		payload[ind++] = COMM_WRITE_NEW_APP_DATA;
		buffer_append_uint32(payload, offset, &ind);
		memcpy(payload + ind, fw_data + offset, chunk);
		ind += (int32_t)chunk;

		/* Reset response state */
		fw_resp_received = false;
		fw_resp_len = 0;

		/* Send chunk */
		comm_can_send_buffer(controller_id, payload, (unsigned int)ind, 0);

		/* Wait for write acknowledgment */
		if (!fw_update_wait_response(timeout_ms)) {
			return FW_UPDATE_ERR_TIMEOUT;
		}

		/* Parse response: [COMM_WRITE_NEW_APP_DATA][success][offset_u32] */
		if (fw_resp_len < 6) {
			return FW_UPDATE_ERR_WRITE;
		}
		if (fw_resp_buf[0] != COMM_WRITE_NEW_APP_DATA) {
			return FW_UPDATE_ERR_WRITE;
		}
		if (fw_resp_buf[1] != 1) {
			return FW_UPDATE_ERR_WRITE;
		}

		/* Verify acknowledged offset */
		int32_t resp_ind = 2;
		uint32_t ack_offset = buffer_get_uint32(fw_resp_buf, &resp_ind);
		if (ack_offset != offset) {
			return FW_UPDATE_ERR_WRITE;
		}

		offset += chunk;

		if (progress_cb) {
			progress_cb(offset, fw_size);
		}
	}

	return FW_UPDATE_OK;
}

/* See vesc_fw_update.h for full documentation. */
fw_update_result_t vesc_fw_update_reboot(uint8_t controller_id)
{
	/* Build COMM_JUMP_TO_BOOTLOADER payload (single byte). */
	uint8_t payload[1];
	payload[0] = COMM_JUMP_TO_BOOTLOADER;

	/* Send with send=2 (no response expected — VESC reboots immediately). */
	comm_can_send_buffer(controller_id, payload, 1, 2);

	return FW_UPDATE_OK;
}

/* See vesc_fw_update.h for full documentation. */
fw_update_result_t vesc_fw_update_full(uint8_t controller_id,
                                       const uint8_t *fw_data,
                                       uint32_t fw_size,
                                       fw_update_progress_cb_t progress_cb)
{
	fw_update_result_t res;

	/* Step 1: Erase flash */
	res = vesc_fw_update_erase(controller_id, fw_size,
	                           FW_UPDATE_ERASE_TIMEOUT_DEFAULT);
	if (res != FW_UPDATE_OK) {
		return res;
	}

	/* Step 2: Write firmware data */
	res = vesc_fw_update_write(controller_id, fw_data, fw_size,
	                           FW_UPDATE_CHUNK_SIZE_DEFAULT,
	                           FW_UPDATE_WRITE_TIMEOUT_DEFAULT,
	                           progress_cb);
	if (res != FW_UPDATE_OK) {
		return res;
	}

	/* Step 3: Reboot into bootloader */
	res = vesc_fw_update_reboot(controller_id);

	return res;
}

/* ── Terminal command ─────────────────────────────────────────────── */

/**
 * @brief Terminal command handler for "fw_update_can".
 *
 * Usage: fw_update_can <controller_id>
 * (Firmware data must be uploaded separately via the host interface.)
 *
 * @param[in] argc  Argument count.
 * @param[in] argv  Argument values.
 *
 * @sideeffects Sends CAN frames. Modifies remote VESC flash. Blocks.
 */
static void terminal_fw_update_can(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("fw_update_can: not yet implemented for terminal usage.");
	commands_printf("Use VESC Tool CAN forwarding for firmware updates.");
}

/* See vesc_fw_update.h for full documentation. */
void vesc_fw_update_terminal_init(void) {
	terminal_register_command_callback(
		"fw_update_can",
		"Remote firmware update over CAN bus",
		"<controller_id>",
		terminal_fw_update_can);
}

/**
 * @brief Hook for capturing CAN firmware update responses.
 *
 * This function should be called from the CAN response dispatch path
 * (commands_send_packet_can_last or equivalent) when a firmware update
 * response is detected.  It captures the response for the waiting
 * firmware update thread.
 *
 * @param[in] data  Response packet data.
 * @param[in] len   Response data length.
 * @return true if the response was consumed, false if not a FW response.
 *
 * @sideeffects Writes to fw_resp_buf, signals fw_waiting_tp.
 */
bool vesc_fw_update_try_capture_response(unsigned char *data, unsigned int len) {
	if (!fw_waiting_tp || len < 1) {
		return false;
	}

	COMM_PACKET_ID pkt = (COMM_PACKET_ID)data[0];
	if (pkt == COMM_ERASE_NEW_APP || pkt == COMM_WRITE_NEW_APP_DATA) {
		fw_update_reply_capture(data, len);
		return true;
	}

	return false;
}
