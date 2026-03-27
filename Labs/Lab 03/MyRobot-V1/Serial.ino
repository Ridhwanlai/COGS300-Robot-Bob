/**
 * @file Serial.ino
 * @brief Minimal serial logging helpers.
 *
 * Very small logging utilities that print tagged messages to an
 * Arduino Stream (e.g. Serial). No wrapping or redefining of serial
 * functions—only uses Stream::print/println.
 *
 * @author Paul Bucci
 * @date 2026
 */

#include <Arduino.h>

/**
 * @brief Print an informational log message.
 *
 * Format:
 *   [INFO] <message>
 *
 * @param out Stream to write to (e.g. Serial)
 * @param msg Null-terminated message string
 */
inline void logInfo(Stream& out, const char* msg) {
  unsigned long timestamp = millis();
  out.print(timestamp);
  out.print(F("\t[INFO] "));
  out.println(msg);
}

/**
 * @brief Print an error log message.
 *
 * Format:
 *   [ERROR] <message>
 *
 * @param out Stream to write to (e.g. Serial)
 * @param msg Null-terminated message string
 */
inline void logError(Stream& out, const char* msg) {
  out.print(F("[ERROR] "));
  out.println(msg);
}
