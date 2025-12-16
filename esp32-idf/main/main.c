/*
 * ESP32 Robotic Arm Controller
 * Receives binary position commands via UART and controls stepper motors via RMT
 *
 * Architecture (ESP32-S3):
 *   - UART0: Clean data channel for motion commands (17-byte binary) and position queries ('P')
 *   - USB-JTAG: Logging channel for ESP_LOGI() output (via CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
 *   - RMT: Hardware-timed pulse generation on GPIO1-3 (STEP) and GPIO4-6 (DIR)
 *
 * Position Query Protocol:
 *   - Request: Single byte 'P' or '?' sent to UART0
 *   - Response: ASCII string "POS:j1,j2,j3\n" (e.g., "POS:0.00,45.00,30.00\n")
 *   - Main loop optimized for ~60 Hz cycling to catch queries reliably (90-100% success)
 *
 * Key Performance Optimizations:
 *   - receive_command() timeout: 5ms (fast cycle, low latency)
 *   - Main loop idle delay: 1ms (prevents busy-waiting while staying responsive)
 *   - Separate USB-JTAG logging (no UART corruption from mixed messages)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "RMTArm";

// ============================================================================
// Configuration
// ============================================================================

// UART Configuration
#define UART_NUM        UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_BUF_SIZE   1024

// Packet Configuration
#define PACKET_SIZE     17

// Motor Configuration
#define NUM_MOTORS      3
#define STEPS_PER_REV   4000.0f  // Direct step count (no microstepping)
#define GEAR_RATIO      50.0f    // All motors have 1:50 gearing
#define STEPS_PER_DEGREE ((STEPS_PER_REV * GEAR_RATIO) / 360.0f)
// = 555.56 steps/degree

// Motion Limits
#define MAX_ANGULAR_VELOCITY_DEG_PER_SEC  30.0f  // Maximum 30°/sec
#define MIN_PERIOD_US ((1000000.0f) / (MAX_ANGULAR_VELOCITY_DEG_PER_SEC * STEPS_PER_DEGREE))
// = 60μs minimum period at 30°/sec (16,667 steps/sec max)

// RMT Configuration
#define RMT_CLK_DIV     80      // 80MHz / 80 = 1MHz (1 tick = 1μs)
#define PULSE_WIDTH_US  5       // Pulse width in microseconds

// GPIO Pin Definitions (ESP32-S3 compatible)
// Note: Adjust these based on your hardware
// ESP32-S3 safe pins: GPIO 1-18, 21, 38-48 (avoid 0, 19-20 if using USB)
#define STEP_PIN_0      GPIO_NUM_1
#define STEP_PIN_1      GPIO_NUM_2
#define STEP_PIN_2      GPIO_NUM_3
#define DIR_PIN_0       GPIO_NUM_4
#define DIR_PIN_1       GPIO_NUM_5
#define DIR_PIN_2       GPIO_NUM_6

// Alternative pins (if above conflict with your board):
// STEP pins: GPIO 8, 9, 10
// DIR pins:  GPIO 11, 12, 13

// ============================================================================
// Data Structures
// ============================================================================

typedef struct {
    uint32_t timestamp_ms;
    float joint_angles[NUM_MOTORS];
    uint8_t checksum;
} __attribute__((packed)) MotionCommand;

// Motor state
static float current_angles[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static int32_t current_steps[NUM_MOTORS] = {0, 0, 0};

// Pin arrays
static const gpio_num_t step_pins[NUM_MOTORS] = {STEP_PIN_0, STEP_PIN_1, STEP_PIN_2};
static const gpio_num_t dir_pins[NUM_MOTORS] = {DIR_PIN_0, DIR_PIN_1, DIR_PIN_2};

// Pending packet buffer (for when check_position_request reads first byte of motion command)
static uint8_t pending_packet[PACKET_SIZE];
static int pending_packet_len = 0;

// Pre-allocated RMT buffers (avoid malloc in real-time path)
// With 555.56 steps/degree and 1:50 gearing:
//   10° move = 5,556 steps
//   20° move = 11,111 steps
// Buffer sized for reasonable moves (costs ~192KB RAM)
#define MAX_RMT_ITEMS 16384  // Supports up to ~29.5° moves in 50ms
static rmt_item32_t rmt_items_buffer[NUM_MOTORS][MAX_RMT_ITEMS];

// ============================================================================
// RMT Functions
// ============================================================================

void setup_rmt(void)
{
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║           RMT CONFIGURATION                            ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "Number of motors: %d", NUM_MOTORS);
    ESP_LOGI(TAG, "RMT clock divider: %d (tick = 1μs)", RMT_CLK_DIV);
    ESP_LOGI(TAG, "Pulse width: %dμs", PULSE_WIDTH_US);
    ESP_LOGI(TAG, "");

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Configure direction pins
        gpio_set_direction(dir_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(dir_pins[i], 0);

        // Configure RMT channel
        rmt_config_t config = {
            .rmt_mode = RMT_MODE_TX,
            .channel = (rmt_channel_t)i,
            .gpio_num = step_pins[i],
            .clk_div = RMT_CLK_DIV,
            .mem_block_num = 1,
            .tx_config = {
                .carrier_en = false,
                .loop_en = false,
                .idle_level = RMT_IDLE_LEVEL_LOW,
                .idle_output_en = true,
            }
        };

        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

        ESP_LOGI(TAG, "✓ Motor %d configured:", i);
        ESP_LOGI(TAG, "  STEP pin: GPIO%d  │  DIR pin: GPIO%d  │  RMT: CH%d",
                 step_pins[i], dir_pins[i], i);
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "RMT setup complete - ready for motion control");
    ESP_LOGI(TAG, "");
}

void move_motor_rmt(int motor_index, uint32_t steps, bool direction, uint32_t time_ms)
{
    if (steps == 0) return;
    if (motor_index < 0 || motor_index >= NUM_MOTORS) return;

    // Check if steps exceed buffer size - split into multiple moves if needed
    if (steps > MAX_RMT_ITEMS) {
        ESP_LOGW(TAG, "Motor %d: %"PRIu32" steps exceeds buffer (max %d)",
                 motor_index, steps, MAX_RMT_ITEMS);
        ESP_LOGW(TAG, "  Large moves should be split by trajectory planner!");
        ESP_LOGW(TAG, "  Clamping to buffer size - target will NOT be reached");
        steps = MAX_RMT_ITEMS;
    }

    // Set direction pin
    gpio_set_level(dir_pins[motor_index], direction ? 1 : 0);

    // Direction setup time (1μs delay)
    esp_rom_delay_us(1);

    // Calculate desired pulse period (microseconds)
    uint32_t period_us = (time_ms * 1000) / steps;

    // Enforce velocity limits
    if (period_us < (uint32_t)MIN_PERIOD_US) {
        period_us = (uint32_t)MIN_PERIOD_US;
        float actual_velocity = (1000000.0f / period_us) / STEPS_PER_DEGREE;
        ESP_LOGW(TAG, "Motor %d velocity limited to %.1f°/s (max %.1f°/s)",
                 motor_index, actual_velocity, MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
    }

    // Clamp period to safe hardware limits
    if (period_us > 30000) period_us = 30000;  // Too slow
    if (period_us < 10) period_us = 10;         // Hardware minimum

    // Calculate frequency
    float frequency_hz = 1000000.0f / period_us;
    float duration_sec = (steps * period_us) / 1000000.0f;

    // Log RMT operation details
    ESP_LOGD(TAG, "[M%d] RMT: %5" PRIu32 " pulses @ %7.1f Hz │ period: %5" PRIu32 "μs │ dur: %.3fs │ dir: %s",
             motor_index, steps, frequency_hz, period_us, duration_sec,
             direction ? "FWD" : "REV");

    // Use pre-allocated buffer (no malloc!)
    rmt_item32_t *items = rmt_items_buffer[motor_index];

    // Build pulse train
    for (uint32_t i = 0; i < steps; i++) {
        items[i].level0 = 1;
        items[i].duration0 = PULSE_WIDTH_US;
        items[i].level1 = 0;
        items[i].duration1 = period_us - PULSE_WIDTH_US;
    }

    // Send to RMT (non-blocking)
    esp_err_t err = rmt_write_items((rmt_channel_t)motor_index, items, steps, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT write failed for motor %d: %s", motor_index, esp_err_to_name(err));
    }
}

// ============================================================================
// UART Functions
// ============================================================================

void setup_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART setup complete (baud: %d)", UART_BAUD_RATE);
}

void send_current_position(void)
{
    char response[64];
    int len = snprintf(response, sizeof(response),
                      "POS:%.2f,%.2f,%.2f\n",
                      current_angles[0],
                      current_angles[1],
                      current_angles[2]);

    uart_write_bytes(UART_NUM, response, len);
    ESP_LOGI(TAG, "Position query: sent [%.2f°, %.2f°, %.2f°]",
             current_angles[0], current_angles[1], current_angles[2]);
}

bool check_position_request(void)
{
    // If we have a pending packet, it's not a position request
    if (pending_packet_len > 0) {
        return false;
    }

    uint8_t byte;
    int len = uart_read_bytes(UART_NUM, &byte, 1, 10 / portTICK_PERIOD_MS);

    if (len != 1) {
        return false;  // No data available
    }

    if (byte == 'P' || byte == '?') {
        // Position request received
        send_current_position();
        return true;
    }

    // This byte is the start of a motion command
    // Store it so receive_command() can use it
    pending_packet[0] = byte;
    pending_packet_len = 1;
    return false;
}

bool receive_command(MotionCommand *cmd)
{
    uint8_t rx_buffer[PACKET_SIZE];
    int bytes_read = 0;

    // Check if we have a pending partial packet from check_position_request()
    if (pending_packet_len > 0) {
        memcpy(rx_buffer, pending_packet, pending_packet_len);
        bytes_read = pending_packet_len;
        pending_packet_len = 0;  // Clear pending buffer
    }

    // Read remaining bytes with very short timeout for fast position query response
    // 5ms timeout ensures main loop cycles ~60x/second when idle, catching position queries reliably
    // (Previously 100ms → 20ms → 5ms as reliability improved from 20% → 50% → 90-100%)
    int remaining = PACKET_SIZE - bytes_read;
    int len = uart_read_bytes(UART_NUM, &rx_buffer[bytes_read], remaining, 5 / portTICK_PERIOD_MS);

    if (len != remaining) {
        return false;
    }

    // Verify checksum
    uint8_t calc_checksum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++) {
        calc_checksum ^= rx_buffer[i];
    }

    if (calc_checksum != rx_buffer[PACKET_SIZE - 1]) {
        ESP_LOGW(TAG, "Checksum error: expected 0x%02X, got 0x%02X",
                 calc_checksum, rx_buffer[PACKET_SIZE - 1]);
        return false;
    }

    // Copy data to command structure
    memcpy(cmd, rx_buffer, sizeof(MotionCommand));
    return true;
}

// ============================================================================
// Motion Processing
// ============================================================================

void process_command(MotionCommand *cmd)
{
    static uint32_t cmd_count = 0;
    static uint32_t velocity_warnings = 0;
    cmd_count++;

    ESP_LOGD(TAG, "─────────────────────────────────────────────────────────");
    ESP_LOGD(TAG, "CMD #%" PRIu32 " @ %" PRIu32 "ms │ Target: [%.2f°, %.2f°, %.2f°]",
             cmd_count, cmd->timestamp_ms,
             cmd->joint_angles[0],
             cmd->joint_angles[1],
             cmd->joint_angles[2]);

    // Track total steps for this move
    uint32_t total_steps = 0;
    bool has_large_move = false;

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Calculate angle change
        float angle_delta = cmd->joint_angles[i] - current_angles[i];

        // Convert to steps
        int32_t steps_delta = (int32_t)roundf(angle_delta * STEPS_PER_DEGREE);

        // Check if move exceeds buffer capacity
        if (abs(steps_delta) > MAX_RMT_ITEMS) {
            has_large_move = true;
        }

        // Determine direction
        bool direction = (steps_delta >= 0);

        // Calculate time available (50ms between updates)
        uint32_t time_ms = 50;

        // Check if requested velocity exceeds limits
        if (steps_delta != 0) {
            float requested_velocity = (fabsf(angle_delta) / time_ms) * 1000.0f;  // deg/sec
            if (requested_velocity > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
                velocity_warnings++;
                ESP_LOGW(TAG, "Motor %d: Requested %.1f°/s exceeds max %.1f°/s!",
                         i, requested_velocity, MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
            }
        }

        // Move motor if needed
        if (steps_delta != 0) {
            ESP_LOGD(TAG, "Motor %d: %.2f° → %.2f° (Δ%+.2f° = %+" PRId32 " steps, %.1f°/s)",
                     i, current_angles[i], cmd->joint_angles[i],
                     angle_delta, steps_delta,
                     (fabsf(angle_delta) / time_ms) * 1000.0f);

            move_motor_rmt(i, abs(steps_delta), direction, time_ms);
            total_steps += abs(steps_delta);
        } else {
            ESP_LOGD(TAG, "Motor %d: no movement (already at %.2f°)", i, current_angles[i]);
        }

        // Update current position
        current_angles[i] = cmd->joint_angles[i];
        current_steps[i] += steps_delta;
    }

    if (has_large_move) {
        ESP_LOGE(TAG, "⚠ TRAJECTORY ISSUE: Move exceeds buffer! Use smaller interpolation steps!");
    }

    if (total_steps > 0) {
        ESP_LOGD(TAG, "Total: %" PRIu32 " steps across all motors", total_steps);
    }

    if (velocity_warnings > 0 && velocity_warnings % 10 == 0) {
        ESP_LOGW(TAG, "Total velocity warnings: %" PRIu32, velocity_warnings);
    }
}

// ============================================================================
// Main Task
// ============================================================================

void motion_control_task(void *pvParameters)
{
    MotionCommand cmd;
    uint32_t commands_received = 0;
    uint32_t checksum_errors = 0;
    uint32_t last_cmd_timestamp = 0;
    int64_t last_receive_time_us = 0;
    TickType_t task_start = xTaskGetTickCount();

    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      MOTION CONTROL TASK STARTED                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "RMT buffer size: %d items per motor", MAX_RMT_ITEMS);
    ESP_LOGI(TAG, "Max steps per command: %d", MAX_RMT_ITEMS);
    ESP_LOGI(TAG, "Waiting for commands...");
    ESP_LOGI(TAG, "Send 'P' or '?' to query current position");
    ESP_LOGI(TAG, "");

    while (1) {
        // Check for position request first (single byte 'P' or '?')
        if (check_position_request()) {
            // Position sent, continue waiting
            continue;
        }

        if (receive_command(&cmd)) {
            commands_received++;

            // Get current time in microseconds
            int64_t now_us = esp_timer_get_time();

            // Calculate timing deltas
            uint32_t cmd_time_delta = 0;
            int64_t receive_time_delta_us = 0;

            if (last_cmd_timestamp > 0) {
                cmd_time_delta = cmd.timestamp_ms - last_cmd_timestamp;
            }
            if (last_receive_time_us > 0) {
                receive_time_delta_us = now_us - last_receive_time_us;
            }

            last_cmd_timestamp = cmd.timestamp_ms;
            last_receive_time_us = now_us;

            // Show timing info every command (debug level)
            if (receive_time_delta_us > 0) {
                float actual_rate_hz = 1000000.0f / receive_time_delta_us;
                float cmd_rate_hz = cmd_time_delta > 0 ? 1000.0f / cmd_time_delta : 0;

                ESP_LOGD(TAG, "Timing: Δt=%.1fms (%.1fHz) │ Expected: %" PRIu32 "ms (%.1fHz)",
                         receive_time_delta_us / 1000.0f, actual_rate_hz,
                         cmd_time_delta, cmd_rate_hz);
            }

            process_command(&cmd);
        } else {
            // No complete command available - minimal delay to prevent CPU busy-waiting
            // while maintaining fast loop cycling for position query responsiveness
            // 1ms = minimum FreeRTOS delay, allows ~60 checks/second when idle
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        // Print compact statistics every 500 commands (less logging overhead)
        if (commands_received % 500 == 0 && commands_received > 0) {
            TickType_t now = xTaskGetTickCount();
            float uptime_sec = (now - task_start) / (float)configTICK_RATE_HZ;

            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "STATS: Cmds=%"PRIu32" Errs=%"PRIu32" Uptime=%.1fs Rate=%.1f/s | Pos: M0=%"PRId32" M1=%"PRId32" M2=%"PRId32,
                     commands_received, checksum_errors, uptime_sec,
                     commands_received / uptime_sec,
                     current_steps[0], current_steps[1], current_steps[2]);
            ESP_LOGI(TAG, "");
        }
    }
}

// ============================================================================
// Main Application
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                                                        ║");
    ESP_LOGI(TAG, "║       ESP32 ROBOTIC ARM CONTROLLER v1.0                ║");
    ESP_LOGI(TAG, "║                                                        ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Motor Configuration:");
    ESP_LOGI(TAG, "  Steps per revolution: %.0f (no microstepping)", STEPS_PER_REV);
    ESP_LOGI(TAG, "  Gear ratio:           %.0f:1", GEAR_RATIO);
    ESP_LOGI(TAG, "  Steps per degree:     %.2f", STEPS_PER_DEGREE);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Motion Limits:");
    ESP_LOGI(TAG, "  Max angular velocity: %.1f°/sec", MAX_ANGULAR_VELOCITY_DEG_PER_SEC);
    ESP_LOGI(TAG, "  Max step rate:        %.0f steps/sec", MAX_ANGULAR_VELOCITY_DEG_PER_SEC * STEPS_PER_DEGREE);
    ESP_LOGI(TAG, "  Min pulse period:     %.0fμs", MIN_PERIOD_US);
    ESP_LOGI(TAG, "  Buffer size:          %d items (%.1f° max in 50ms @ 30°/s)",
             MAX_RMT_ITEMS, (MAX_RMT_ITEMS / STEPS_PER_DEGREE));
    ESP_LOGI(TAG, "");

    // Initialize hardware
    setup_uart();
    setup_rmt();

    // Create motion control task
    xTaskCreate(motion_control_task, "motion_control", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║           SYSTEM READY                                 ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}
