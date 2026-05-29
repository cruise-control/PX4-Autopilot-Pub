# Rotary encoder event message
# Published by the ec11_rotary_encoder driver

uint64 timestamp          # Time of the event [us]

int32  position           # Absolute encoder position (cumulative steps)
int8   delta              # Change since last publish (+1 CW, -1 CCW)

bool   button_push        # Encoder shaft push-button pressed (PUSH pin)
bool   button_k0          # Independent K0/back button pressed

# Convenience fields
bool   push_rising        # True on press edge
bool   push_falling       # True on release edge
bool   k0_rising
bool   k0_falling
