# Display command message
# Subscribed to by the st7789_display driver

uint64 timestamp          # Time command was generated [us]

# ---- Numeric output (large centre area) ----
float32 numeric_value     # Value to display
uint8   decimal_places    # Decimal places to show (0-4)
char[8] units             # Unit string e.g. "RPM\0", "m/s\0"

# ---- Status line (top bar) ----
char[32] status_text      # Status message (null-terminated)
uint8    status_color     # 0=white 1=green 2=yellow 3=red

# ---- Display control ----
bool     backlight_on     # Backlight enable
uint8    brightness       # PWM brightness 0-100 (if BLK wired)
bool     clear_display    # Force full redraw / clear screen
