/*
 ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include "ch.h"
#include "hal.h"

#include <sim900d.h>
#include <mavlink.h>

#define SIM900D_DETECTED 	0x01
#define SIM900D_INIT_ERROR 	0x02

#define SIM900D_SEND_BUF MAVLINK_MAX_PACKET_LEN * 2
#define SIM900D_SEND_TIMEOUT_MS	500

uint8_t g_boardStatus = 0;

static const SerialConfig uart2_cfg = { 115200, LCR_WL8 | LCR_STOP1
		| LCR_NOPARITY, FCR_TRIGGER0 };
static const SerialConfig uart3_cfg = { 57600, LCR_WL8 | LCR_STOP1
		| LCR_NOPARITY, FCR_TRIGGER0 };

static WORKING_AREA(waBlinkerThread, 64);
static msg_t BlinkerThread(void *arg) {
	(void) arg;
	palClearPad(GPIO1, GPIO1_LED2_Y);palClearPad(GPIO1, GPIO1_LED1_Y);
	while (TRUE) {
		systime_t time = 50;
		if (g_boardStatus & SIM900D_DETECTED)
			time = 250;
		else if (g_boardStatus & SIM900D_INIT_ERROR)
			time = 2000;
		else
			time = 50;
		palTogglePad(GPIO1, GPIO1_LED2_Y);
		chThdSleepMilliseconds(time);
	}
	/* This point may be reached if shut down is requested. */
	return 0;
}

static WORKING_AREA(waUARTResend, 2048);
static msg_t UARTResend(void *arg) {
	(void) arg;
	uint8_t buf[64];
	while (TRUE) {
		uint8_t bytesRead = chnReadTimeout(&SD3, buf, 64, MS2ST(5));
		if (bytesRead > 0)
			chnWrite(&SD4, buf, bytesRead);
		bytesRead = chnReadTimeout(&SD4, buf, 64, MS2ST(5));
		if (bytesRead > 0)
			chnWrite(&SD3, buf, bytesRead);
		//chnWrite(&SD4, b, sizeof(b));
		chThdSleepMilliseconds(100);
	}
	/* This point may be reached if shut down is requested. */
	return 0;
}

static WORKING_AREA(waUART, 6144);
static msg_t UART(void *arg) {
	(void) arg;
	if (init_sim900d(&SD3)) {
		g_boardStatus |= SIM900D_DETECTED;
	} else
		g_boardStatus |= SIM900D_INIT_ERROR;

	static mavlink_message_t msg;
	static mavlink_message_t msgs;
	static mavlink_status_t status;
	uint8_t bufS[SIM900D_SEND_BUF];
	uint8_t bufS1[MAVLINK_MAX_PACKET_LEN];
	uint16_t total_len = 0;
	uint8_t mavBuff[64];
	systime_t last_send = 0;

	while (g_boardStatus & SIM900D_DETECTED) {
		uint8_t bytesRead = chnReadTimeout(&SD4, mavBuff, 64, MS2ST(5));
		uint8_t i;
		for (i = 0; i < bytesRead; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_0, mavBuff[i], &msg, &status)) {
				switch(msg.msgid){
					case MAVLINK_MSG_ID_SYS_STATUS: {
						mavlink_sys_status_t pack;
						mavlink_msg_sys_status_decode(&msg, &pack);
						mavlink_msg_sys_status_pack(msg.sysid, msg.compid, &msgs,
								pack.onboard_control_sensors_present, pack.onboard_control_sensors_enabled,
								pack.onboard_control_sensors_health, pack.load,
								pack.voltage_battery, pack.current_battery, pack.battery_remaining,
								pack.drop_rate_comm, pack.errors_comm, pack.errors_count1,
								pack.errors_count2, pack.errors_count3, pack.errors_count4);
						break;
					}
					case MAVLINK_MSG_ID_ATTITUDE: {
						mavlink_attitude_t pack;
						mavlink_msg_attitude_decode(&msg, &pack);
						mavlink_msg_attitude_pack(msg.sysid, msg.compid, &msgs,
								pack.time_boot_ms, pack.roll, pack.pitch, pack.yaw,
								pack.rollspeed, pack.pitchspeed, pack.yawspeed);
						break;
					}
					case MAVLINK_MSG_ID_VFR_HUD: {
						mavlink_vfr_hud_t pack;
						mavlink_msg_vfr_hud_decode(&msg, &pack);
						mavlink_msg_vfr_hud_pack(msg.sysid, msg.compid, &msgs,
								pack.airspeed, pack.groundspeed, pack.heading, pack.throttle,
								pack.alt, pack.climb);
						break;
					}
					case MAVLINK_MSG_ID_SYSTEM_TIME: {
						mavlink_system_time_t pack;
						mavlink_msg_system_time_decode(&msg, &pack);
						mavlink_msg_system_time_pack(msg.sysid, msg.compid, &msgs,
								pack.time_unix_usec, pack.time_boot_ms);
					}
					/*case MAVLINK_MSG_ID_BATTERY_STATUS: {
						mavlink_battery_status_t pack;
						mavlink_msg_battery_status_decode(&msg, &pack);
						mavlink_msg_battery_status_pack(msg.sysid, msg.compid, &msgs,
								pack.id, pack.battery_function, pack.type, pack.temperature,
								pack.voltages, pack.current_battery, pack.current_consumed,
								pack.energy_consumed, pack.battery_remaining);
						break;
					}*/
					case MAVLINK_MSG_ID_COMMAND_ACK: {
						mavlink_command_ack_t pack;
						mavlink_msg_command_ack_decode(&msg, &pack);
						mavlink_msg_command_ack_pack(msg.sysid, msg.compid, &msgs,
								pack.command, pack.result);
						break;
					}
					case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
					    mavlink_global_position_int_t pack;
					    mavlink_msg_global_position_int_decode(&msg, &pack);
					    mavlink_msg_global_position_int_pack(msg.sysid, msg.compid, &msgs,
					            pack.time_boot_ms, pack.lat, pack.lon, pack.alt,
					            pack.relative_alt, pack.vx, pack.vy, pack.vz, pack.hdg);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_COUNT: {
					    mavlink_mission_count_t pack;
					    mavlink_msg_mission_count_decode(&msg, &pack);
					    mavlink_msg_mission_count_pack(msg.sysid, msg.compid, &msgs,
					            pack.target_system, pack.target_component, pack.count);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_ACK: {
					    mavlink_mission_ack_t pack;
					    mavlink_msg_mission_ack_decode(&msg, &pack);
					    mavlink_msg_mission_ack_pack(msg.sysid, msg.compid, &msgs,
					            pack.target_system, pack.target_component, pack.type);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_ITEM: {
					    mavlink_mission_item_t pack;
					    mavlink_msg_mission_item_decode(&msg, &pack);
					    mavlink_msg_mission_item_pack(msg.sysid, msg.compid, &msgs,
                                pack.target_system, pack.target_component, pack.seq,
                                pack.frame, pack.command, pack.current, pack.autocontinue,
                                pack.param1, pack.param2, pack.param3, pack.param4,
                                pack.x, pack.y, pack.z);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_CURRENT: {
					    mavlink_mission_current_t pack;
					    mavlink_msg_mission_current_decode(&msg, &pack);
					    mavlink_msg_mission_current_pack(msg.sysid, msg.compid, &msgs,
					            pack.seq);
					    break;
					}
					case MAVLINK_MSG_ID_STATUSTEXT: {
					    mavlink_statustext_t pack;
					    mavlink_msg_statustext_decode(&msg, &pack);
					    mavlink_msg_statustext_pack(msg.sysid, msg.compid, &msgs,
					            pack.severity, pack.text);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST: {
					    mavlink_mission_request_partial_list_t pack;
					    mavlink_msg_mission_request_partial_list_decode(&msg, &pack);
					    mavlink_msg_mission_request_partial_list_pack(msg.sysid,
					            msg.compid, &msgs, pack.target_system, pack.target_component,
					            pack.start_index, pack.end_index);
					    break;
					}
					case MAVLINK_MSG_ID_MISSION_REQUEST: {
					    mavlink_mission_request_t pack;
					    mavlink_msg_mission_request_decode(&msg, &pack);
					    mavlink_msg_mission_request_pack(msg.sysid, msg.compid, &msgs,
					            pack.target_system, pack.target_component, pack.seq);

					}
					case MAVLINK_MSG_ID_COMMAND_INT: {
					    mavlink_command_int_t pack;
					    mavlink_msg_command_int_decode(&msg, &pack);
					    mavlink_msg_command_int_pack(msg.sysid, msg.compid, &msgs,
					            pack.target_system, pack.target_component, pack.frame,
					            pack.command, pack.current, pack.autocontinue,
					            pack.param1, pack.param2, pack.param3, pack.param4,
					            pack.x, pack.y, pack.z);
					    break;
					}
                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t pack;
                        mavlink_msg_command_long_decode(&msg, &pack);
                        mavlink_msg_command_long_pack(msg.sysid, msg.compid,
                                &msgs, pack.target_system,
                                pack.target_component, pack.command, pack.confirmation,
                                pack.param1, pack.param2, pack.param3, pack.param4,
                                pack.param5, pack.param6, pack.param7);
                        break;
                    }
					case MAVLINK_MSG_ID_HEARTBEAT: {
						mavlink_heartbeat_t pack;
						mavlink_msg_heartbeat_decode(&msg, &pack);
						mavlink_msg_heartbeat_pack(msg.sysid, msg.compid, &msgs,
								pack.type, pack.autopilot, pack.base_mode, pack.custom_mode,
								pack.system_status);
						break;
					}
				}
				if (total_len + (uint16_t) (MAVLINK_NUM_NON_PAYLOAD_BYTES + msgs.len)>= SIM900D_SEND_BUF) {
					chnWrite(&SD3, bufS, total_len);
					total_len = 0;
					last_send = MS2ST(chTimeNow());
				}
				uint16_t len = mavlink_msg_to_send_buffer(&bufS[total_len], &msgs);
				total_len += len;
			}
		}
		if (MS2ST(chTimeNow()) > last_send + SIM900D_SEND_TIMEOUT_MS
				&& total_len > 0) {
			chnWrite(&SD3, bufS, total_len);
			total_len = 0;
			last_send = MS2ST(chTimeNow());
		}

		bytesRead = chnReadTimeout(&SD3, mavBuff, 64, MS2ST(5));
		for (i = 0; i < bytesRead; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_1, mavBuff[i], &msg, &status)) {
				uint16_t len = mavlink_msg_to_send_buffer(bufS1, &msg);
				chnWrite(&SD4, bufS1, len);
			}
		}
		chThdSleepMilliseconds(20);
	}
	/* This point may be reached if shut down is requested. */
	return 0;
}

/*
 * Application entry point.
 */
int main(void) {

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	sdStart(&SD3, &uart2_cfg); //UART2 for GSM
	sdStart(&SD4, &uart3_cfg); //UART3 for telemetry

	chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread),
	NORMALPRIO - 1, BlinkerThread, NULL);

	chThdSleepMilliseconds(500);

	chThdCreateStatic(waUART, sizeof(waUART),
	NORMALPRIO, UART, NULL);

	/*reset_sim900d();
	 chThdCreateStatic(waUARTResend, sizeof(waUARTResend),
	 NORMALPRIO, UARTResend, NULL);*/

	while (TRUE) {
		chThdSleepMilliseconds(1000);
	}
}
