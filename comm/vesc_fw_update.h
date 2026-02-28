/*
	Copyright 2026 RC-Plane-Project Contributors

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
 * @file vesc_fw_update.h
 * @brief Remote VESC firmware update over CAN bus.
 *
 * Provides functions to erase, write, and reboot the firmware on a remote
 * VESC ESC over the CAN bus.  Uses the standard VESC CAN multi-frame
 * transport (comm_can_send_buffer) and the COMM_ERASE_NEW_APP /
 * COMM_WRITE_NEW_APP_DATA / COMM_JUMP_TO_BOOTLOADER protocol.
 *
 * This module runs on the VESC that initiates the update (the "sender"),
 * not on the VESC being updated (the "target").
 *
 * The firmware binary must be pre-processed with VESC Tool --packFirmware
 * before upload.
 *
 * WARNING: Do NOT disconnect power during firmware upload.
 */

#ifndef VESC_FW_UPDATE_H
#define VESC_FW_UPDATE_H

#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Result codes for firmware update operations.
 */
typedef enum {
	FW_UPDATE_OK             = 0,
	FW_UPDATE_ERR_ERASE      = 1,
	FW_UPDATE_ERR_WRITE      = 2,
	FW_UPDATE_ERR_TIMEOUT    = 3,
	FW_UPDATE_ERR_PARAM      = 4,
} fw_update_result_t;

/**
 * @brief Progress callback for firmware update.
 *
 * @param[in] bytes_written  Bytes written so far.
 * @param[in] bytes_total    Total firmware size.
 */
typedef void (*fw_update_progress_cb_t)(uint32_t bytes_written,
                                        uint32_t bytes_total);

/**
 * @brief Erase the firmware flash on a remote VESC over CAN.
 *
 * Sends COMM_ERASE_NEW_APP with the firmware size and waits for the
 * acknowledgment.  Flash erase takes several seconds.
 *
 * @param[in] controller_id  Target VESC CAN controller ID.
 * @param[in] fw_size        Total firmware size in bytes.
 * @param[in] timeout_ms     Maximum wait time for erase ack (ms).
 * @return FW_UPDATE_OK on success, or an error code.
 *
 * @sideeffects Transmits CAN frames.  Blocks for up to timeout_ms.
 */
fw_update_result_t vesc_fw_update_erase(uint8_t controller_id,
                                        uint32_t fw_size,
                                        uint32_t timeout_ms);

/**
 * @brief Write firmware data to a remote VESC over CAN.
 *
 * Sends COMM_WRITE_NEW_APP_DATA in chunks.  Each chunk contains a
 * 4-byte big-endian offset followed by firmware data (max ~500 bytes).
 * Waits for acknowledgment after each chunk.
 *
 * @param[in] controller_id  Target VESC CAN controller ID.
 * @param[in] fw_data        Pointer to the packed firmware binary.
 * @param[in] fw_size        Total firmware size in bytes.
 * @param[in] chunk_size     Bytes per chunk (max 500, recommended).
 * @param[in] timeout_ms     Per-chunk acknowledgment timeout (ms).
 * @param[in] progress_cb    Optional progress callback (may be NULL).
 * @return FW_UPDATE_OK on success, or an error code.
 *
 * @sideeffects Transmits CAN frames.  Blocks for the duration of upload.
 */
fw_update_result_t vesc_fw_update_write(uint8_t controller_id,
                                        const uint8_t *fw_data,
                                        uint32_t fw_size,
                                        uint32_t chunk_size,
                                        uint32_t timeout_ms,
                                        fw_update_progress_cb_t progress_cb);

/**
 * @brief Reboot a remote VESC into the bootloader.
 *
 * Sends COMM_JUMP_TO_BOOTLOADER.  The target VESC reboots immediately
 * and does not send an acknowledgment.  Wait at least 10 seconds before
 * attempting to communicate again.
 *
 * @param[in] controller_id  Target VESC CAN controller ID.
 * @return FW_UPDATE_OK on success, FW_UPDATE_ERR_PARAM on bad input.
 *
 * @sideeffects Transmits CAN frames.  Target VESC reboots.
 */
fw_update_result_t vesc_fw_update_reboot(uint8_t controller_id);

/**
 * @brief Run a complete firmware update: erase, write, reboot.
 *
 * Convenience function that calls vesc_fw_update_erase,
 * vesc_fw_update_write, and vesc_fw_update_reboot in sequence.
 *
 * @param[in] controller_id   Target VESC CAN controller ID.
 * @param[in] fw_data         Pointer to the packed firmware binary.
 * @param[in] fw_size         Total firmware size in bytes.
 * @param[in] progress_cb     Optional progress callback (may be NULL).
 * @return FW_UPDATE_OK on success, or the first error encountered.
 *
 * @sideeffects Transmits CAN frames.  Erases and writes target flash.
 *              Target VESC reboots on success.
 *              Blocks for several minutes.
 */
fw_update_result_t vesc_fw_update_full(uint8_t controller_id,
                                       const uint8_t *fw_data,
                                       uint32_t fw_size,
                                       fw_update_progress_cb_t progress_cb);

/**
 * @brief Register terminal commands for firmware update.
 *
 * Registers "fw_update_can" terminal command that allows triggering
 * firmware updates from the VESC Tool terminal.
 *
 * @sideeffects Registers terminal command callbacks.
 */
void vesc_fw_update_terminal_init(void);

/**
 * @brief Hook for capturing CAN firmware update responses.
 *
 * Call from the CAN response dispatch path when a response arrives
 * from a remote VESC.  If a firmware update is in progress and the
 * response is a COMM_ERASE_NEW_APP or COMM_WRITE_NEW_APP_DATA ack,
 * this function captures it and signals the waiting thread.
 *
 * @param[in] data  Response packet data (starts with COMM_PACKET_ID).
 * @param[in] len   Response data length.
 * @return true if the response was consumed, false otherwise.
 *
 * @sideeffects Writes to internal response buffer, signals waiting thread.
 */
bool vesc_fw_update_try_capture_response(unsigned char *data, unsigned int len);

#endif /* VESC_FW_UPDATE_H */
