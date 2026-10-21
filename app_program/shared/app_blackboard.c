/** @file app_blackboard.c
 *  @brief Storage for every published blackboard word. Each word has exactly
 *         one writing task (see the owning header); this file only provides
 *         aligned storage and the failsafe initial values (Rev B
 *         "publish failsafe initial state").
 */

#include "app_barcode.h"
#include "app_imu.h"
#include "app_line.h"
#include "app_motion.h"
#include "app_obstacle.h"
#include "app_telemetry.h"

/* Motion: BRAKED, zero speed, no commands done. Reads as state 0 = BRAKED. */
volatile uint32_t g_motion_status_a = 0U;
volatile uint32_t g_motion_status_b = 0U;
volatile uint16_t g_enc_ticks_l     = 0U;
volatile uint16_t g_enc_ticks_r     = 0U;

/* Line: not on line, no event, seq 0 (readers treat seq 0 + no publish as
 * stale). */
volatile uint32_t g_line_state = 0U;

/* Barcode. */
app_edge_ring_t  g_edge_ring    = {{0U}, 0U, 0U, 0U, 0U};
volatile uint8_t g_barcode_byte = 0U; /* APP_BC_NONE, seq 0 */

/* IMU: quality INVALID until the first valid sample is published. */
volatile uint32_t g_imu_state =
    ((uint32_t) APP_HUMP_Q_INVALID << APP_IS_QUALITY_SHIFT);

/* Obstacle: range sentinel 511 (no echo), action STOP. */
volatile uint32_t g_obstacle_profile[APP_OBSTACLE_WORDS] = {
    ((uint32_t) APP_RANGE_CM_SENTINEL << APP_OP0_RANGE_SHIFT),
    ((uint32_t) APP_RANGE_CM_SENTINEL << APP_OP1_CLOSEST_SHIFT)};
volatile uint16_t g_echo_width_us = 0U;
volatile uint8_t  g_echo_status   = APP_ECHO_IDLE;

/* Controller / comms. S0, no pending command. */
volatile uint32_t g_mission_status = 0U;
volatile uint16_t g_comms_word = 0U; /* WIFI_CONNECTING, backoff 0, dropped 0 */
volatile uint8_t  g_remote_cmd = 0U;
volatile uint8_t  g_remote_cmd_gen = 0U;

/* Size and alignment guards (NFR28 evidence, R9). */
APP_STATIC_ASSERT(sizeof(g_motion_status_a) == 4U, motion_a_is_one_word);
APP_STATIC_ASSERT(sizeof(g_motion_status_b) == 4U, motion_b_is_one_word);
APP_STATIC_ASSERT(sizeof(g_line_state) == 4U, line_state_is_one_word);
APP_STATIC_ASSERT(sizeof(g_imu_state) == 4U, imu_state_is_one_word);
APP_STATIC_ASSERT(sizeof(g_mission_status) == 4U, mission_status_is_one_word);
APP_STATIC_ASSERT(sizeof(g_obstacle_profile) == 8U,
                  obstacle_profile_is_two_words);
APP_STATIC_ASSERT(sizeof(g_comms_word) == 2U, comms_word_is_halfword);
APP_STATIC_ASSERT(sizeof(g_edge_ring.entries) == APP_EDGE_RING_BYTES,
                  edge_ring_128_bytes);
APP_STATIC_ASSERT(sizeof(g_enc_ticks_l) == 2U, enc_ticks_halfword);
APP_STATIC_ASSERT(sizeof(g_echo_width_us) == 2U, echo_width_halfword);
