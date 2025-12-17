# Logging Configuration for ESP32-WROOM

This document explains the logging configuration optimized for ESP32-WROOM's shared UART architecture.

## The Problem: Shared UART

Unlike ESP32-S3 which has separate USB-JTAG for logging, **ESP32-WROOM has only UART0** for both:
- Data communication (motion commands and position queries)
- System logging (ESP_LOGI, ESP_LOGW, ESP_LOGE)

This means log messages can interfere with binary data packets and position query responses.

## The Solution: Selective Logging

### Position Query Logging - DISABLED

**Location:** `send_current_position()` in main.c

**Before:**
```c
uart_write_bytes(UART_NUM, response, len);
ESP_LOGI(TAG, "Position query: sent [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]", ...);
```

**UART output:**
```
I (12345) RMTArm: Position query: sent [0.00°, 0.00°, 0.00°, 0.00°, 0.00°, 0.00°]
POS:0.00,0.00,0.00,0.00,0.00,0.00
```
❌ Python parser receives mixed log + data, causing parsing errors.

**After:**
```c
uart_write_bytes(UART_NUM, response, len);
// Log removed to prevent UART interference (shared channel on WROOM)
```

**UART output:**
```
POS:0.00,0.00,0.00,0.00,0.00,0.00
```
✅ Python parser receives clean position data only.

### Command Logging - DEBUG Level Only

**Location:** `process_command()` in main.c

All per-command details are set to `ESP_LOGD()` (DEBUG level):
- Command headers
- Target angles
- Per-motor movement details
- Timing information
- Step counts

**By default (INFO level):** These are hidden, reducing UART traffic.

**When needed:** Enable via `idf.py menuconfig` → Component config → Log output → Debug

### What's Still Logged (INFO/WARN/ERROR)

**Startup information:**
- Motor configuration
- GPIO pin assignments
- RMT channel setup
- System ready message

**Runtime issues:**
- Velocity limit warnings
- Buffer overflow errors
- Checksum errors

**Periodic statistics (every 500 commands):**
```
STATS: Cmds=500 Errs=0 Uptime=25.3s Rate=19.8/s
  Pos: M0=12345 M1=12345 M2=12345 M3=12345 M4=12345 M5=12345
```

## Impact on Debugging

### Recommended Workflow

**1. Normal Operation:**
- Keep INFO level logging (default)
- Clean position queries
- Minimal UART traffic
- Python controller works reliably

**2. Debugging Issues:**
- Enable DEBUG logging temporarily
- See detailed command and motor information
- **Warning:** Position queries may be less reliable due to UART traffic
- Disable DEBUG when done

### How to Enable DEBUG Logging

**Option 1: menuconfig**
```bash
idf.py menuconfig
# → Component config
# → Log output
# → Default log verbosity
# → [Select] Debug
# Save and rebuild
```

**Option 2: sdkconfig**
Add to `sdkconfig` or `sdkconfig.defaults`:
```
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
```

**Option 3: Runtime (if enabled in menuconfig)**
```c
esp_log_level_set("RMTArm", ESP_LOG_DEBUG);
```

## Position Query Reliability

### With Logging Disabled (Current Configuration)

**Expected behavior:**
- 90-100% position query success rate
- Clean ASCII responses
- Python retry mechanism (5 attempts) almost always succeeds
- No UART corruption from log messages

### If Logging Were Enabled

**What would happen:**
- Log messages intermixed with position responses
- Python parser needs to filter out log lines
- Still works due to filtering, but less reliable
- More retries needed
- Potential for data corruption if timing is unlucky

## For Debugging Position Queries

If you need to verify position queries are being received:

**Method 1: Temporary DEBUG log**
```c
void send_current_position(void)
{
    // ... send response ...

    ESP_LOGD(TAG, "Position query: sent [%.2f°, ...]", current_angles[0]);
    // DEBUG level - won't show unless enabled
}
```

**Method 2: Increment counter**
```c
static uint32_t query_count = 0;

void send_current_position(void)
{
    // ... send response ...

    query_count++;
    if (query_count % 100 == 0) {
        ESP_LOGI(TAG, "Position queries received: %"PRIu32, query_count);
    }
}
```

**Method 3: Use idf.py monitor with filtering**
```bash
idf.py monitor | grep "POS:"
# Shows only position responses, filters out other logs
```

## Comparison: WROOM vs S3

| Feature | ESP32-WROOM | ESP32-S3 |
|---------|-------------|----------|
| UART channels | UART0 only | UART0 + USB-JTAG |
| Position query logging | Disabled (clean data) | Can be enabled safely |
| Debug logging impact | High (UART interference) | Low (separate channel) |
| Position query reliability | 90-100% (with logging off) | 99%+ (any log level) |
| Recommended log level | INFO | DEBUG (if needed) |

## Summary

**Key takeaways:**
1. ✅ Position query logging is **disabled** to ensure clean UART data
2. ✅ Per-command details are **DEBUG level only** (hidden by default)
3. ✅ Critical errors and warnings are still visible
4. ✅ This configuration maximizes position query reliability on WROOM
5. ⚠️ Enable DEBUG logging only when actively debugging
6. ⚠️ Remember to disable DEBUG after troubleshooting

**The result:** Clean, reliable position queries with minimal UART interference on ESP32-WROOM's shared serial channel.
