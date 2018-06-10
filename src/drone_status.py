class DroneStatus(object):

    state_landed = 0  # Landed state
    state_takingoff = 1  # Taking off state
    state_hovering = 2  # Hovering / Circling (for fixed wings) state
    state_flying = 3  # Flying state
    state_landing = 4  # Landing state
    state_emergency = 5  # Emergency state
    state_usertakeoff = 6  # User take off state. Waiting for user action to take off.
    state_motor_ramping = 7  # Motor ramping state (for fixed wings).
    state_emergency_landing = 8  # Emergency landing state. Drone autopilot has detected