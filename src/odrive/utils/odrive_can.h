#ifndef ODRIVE_CAN
#define ODRIVE_CAN

#include <Arduino.h>
#include <driver/twai.h>
#include <config.h>

#define MOTOR_ANGLE_TO_RAD(MOTOR_TURNS) (MOTOR_TURNS / 30 * 2 * PI)
#define MOTOR_ANGLE_TO_DEG(MOTOR_TURNS) (MOTOR_TURNS / 30 * 360)

// refer to https://docs.odriverobotics.com/can-protocol
#define ODRIVE_MSG_ID(AXIS_ID, CMD_ID) ((AXIS_ID << 5) | CMD_ID)
#define ODRIVE_AXIS_ID(MSG_ID) (MSG_ID >> 5)
#define ODRIVE_CMD_ID(MSG_ID) (MSG_ID & 0x1F)

// Command IDs
#define ODRIVE_CMD_CANOPEN_NMT 0x00
#define ODRIVE_CMD_HEARTBEAT 0x01
#define ODRIVE_CMD_ESTOP 0x02
#define ODRIVE_CMD_GET_MOTOR_ERROR 0x03
#define ODRIVE_CMD_GET_ENCODER_ERROR 0x04
#define ODRIVE_CMD_GET_CONTROLLER_ERROR 0x1D
#define ODRIVE_CMD_GET_SENSORLESS_ERROR 0x05
#define ODRIVE_CMD_SET_AXIS_NODE_ID 0x06
#define ODRIVE_CMD_SET_REQUESTED_STATE 0x07
#define ODRIVE_CMD_SET_STARTUP_CONFIG 0x08
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x09
#define ODRIVE_CMD_GET_ENCODER_COUNT 0x0A
#define ODRIVE_CMD_SET_CONTROLLER_MODES 0x0B
#define ODRIVE_CMD_SET_INPUT_POS 0x0C
#define ODRIVE_CMD_SET_INPUT_VEL 0x0D
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x0E
#define ODRIVE_CMD_SET_LIMITS 0x0F
#define ODRIVE_CMD_START_ANTICOGGING 0x10
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT 0x11
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITs 0x12
#define ODRIVE_CMD_SET_TRAJ_INERTIA 0x13
#define ODRIVE_CMD_GET_IQ 0x14
#define ODRIVE_CMD_GET_SENSORLESS_ESTIMATES 0x15
#define ODRIVE_CMD_REBOOT 0x16
#define ODRIVE_CMD_GET_VBUS_VOLTAGE 0x17
#define ODRIVE_CMD_CLEAR_ERRORS 0x18
#define ODRIVE_CMD_SET_LINEAR_COUNT 0x19
#define ODRIVE_CMD_CANOPEN_HEARTBEAT 0x700

// Protocol Constants
typedef enum
{
    ODRIVE_AXIS_STATE_UNDEFINED = 0,
    ODRIVE_AXIS_STATE_IDLE = 1,
    ODRIVE_AXIS_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
    // missing 5
    ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    ODRIVE_AXIS_STATE_LOCKIN_SPIN = 9,
    ODRIVE_AXIS_STATE_ENCODER_DIR_FIND = 10,
    ODRIVE_AXIS_STATE_HOMING = 11,
    ODRIVE_AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    ODRIVE_AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
} ODriveAxisState;

typedef enum
{
    ODRIVE_AXIS_ERROR_INVALID_STATE = 0x1,
    ODRIVE_AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x800,
    ODRIVE_AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x1000,
    ODRIVE_AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x2000,
    ODRIVE_AXIS_ERROR_ESTOP_REQUESTED = 0x4000,
    ODRIVE_AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x20000,
    ODRIVE_AXIS_ERROR_OVER_TEMP = 0x40000,
    ODRIVE_AXIS_ERROR_UNKNOWN_POSITION = 0x80000,
} ODriveAxisError;

typedef enum
{
    ODRIVE_CONTROL_MODE_VOLTAGE = 0,
    ODRIVE_CONTROL_MODE_TORQUE = 1,
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_CONTROL_MODE_POSITION = 3,
} ODriveControlMode;

typedef enum
{
    ODRIVE_INPUT_MODE_INACTIVE = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP = 2,
    ODRIVE_INPUT_MODE_POS_FILTER = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
    ODRIVE_INPUT_MODE_MIRROR = 7,
    ODRIVE_INPUT_MODE_TUNING = 8,
} ODriveInputMode;

// Protocol Messages

// position in units of turns
// velocity in units of turns per second
// torque in units of Nm
// refer to
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/odrive-interface.yaml

typedef struct
{
    uint32_t axis_error;
    uint8_t axis_current_state;
    bool motor_error_flag : 1;
    bool encoder_error_flag : 1;
    bool controller_error_flag : 1;
    bool trajectory_done_flag : 1;
} ODriveHeartbeat;

typedef struct
{
    float position;
    float velocity;
} ODriveEncoderEstimates;

typedef struct
{
    int32_t shadow_count;
    int32_t count_cpr;
} ODriveEncoderCount;

typedef struct
{
    int32_t control_mode;
    int32_t input_mode;
} ODriveControllerModes;

typedef struct
{
    float position;
    int16_t velocity_x1000;
    int16_t torque_x1000;
} ODriveInputPosition;

typedef struct
{
    float velocity;
    float torque;
} ODriveInputVelocity;

typedef struct
{
    float velocity_limit;
    float current_limit;
} ODriveInputLimits;

typedef struct
{
    float accel_limit;
    float decel_limit;
} ODriveTrajAccelLimit;

typedef struct
{
    float setpoint;
    float measured;
} ODriveIq;

typedef ODriveEncoderEstimates ODriveSensorlessEstimates;

typedef struct
{
    bool motor_error : 1;
    bool encoder_error : 1;
    bool sensorless_error : 1;
    bool controller_error : 1;
    bool heartbeat : 1;
    bool encoder_count : 1;
    bool encoder_estimates : 1;
    bool sensorless_estimates : 1;
    bool iq : 1;
    bool vbus_voltage : 1;
    bool reset_updates : 1;
} ODriveUpdates;

typedef struct
{
    uint64_t motor_error;
    uint32_t encoder_error;
    uint32_t controller_error;
    uint32_t sensorless_error;
    ODriveHeartbeat heartbeat;
    ODriveEncoderCount encoder_count;
    ODriveEncoderEstimates encoder_estimates;
    ODriveSensorlessEstimates sensorless_estimates;
    ODriveIq iq;
    float vbus_voltage;
    ODriveUpdates updates;

    void (*state_transition_callback)(
        uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void *context);
    void *state_transition_context;
} ODriveAxis;

extern ODriveAxis axes[AXIS_COUNT];

static void log_msg(const twai_message_t *msg, const char *str)
{
    Serial.printf("ODriveCan %s (id %x flags %x len %d)\n", str, msg->identifier, msg->flags,
                  msg->data_length_code);
}

void wait_til_heartbeat(int id)
{
    while (!axes[id].updates.heartbeat)
    {
        // display.clearDisplay();
        // display.setCursor(0,0);
        Serial.printf("Waiting for heartbeat %d... Heartbeat: %d\n", id, axes[id].updates.heartbeat);
        // display.printf("%d.No heartbeat\n", id);
        // display.display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Public Functions
esp_err_t odrive_send_get_command(uint8_t axis_id, uint8_t cmd_id)
{
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, cmd_id);
    msg.rtr = 1;
    return twai_transmit(&msg, 0);
}

esp_err_t odrive_send_command(uint8_t axis_id, uint8_t cmd_id, const void *buf, uint8_t len)
{
    twai_message_t msg = {};
    msg.identifier = ODRIVE_MSG_ID(axis_id, cmd_id);
    msg.data_length_code = len;
    memcpy(msg.data, buf, len);
    
    return twai_transmit(&msg, 0);
}

bool odrive_update(ODriveAxis *axes, uint8_t len, twai_message_t msg)
{
    assert(axes);
    assert(len < 32);
    bool error;

    if (msg.flags)
    {
        log_msg(&msg, "received message with invalid flag");
        return true;
    }
    // check axis id
    uint8_t axis_id = ODRIVE_AXIS_ID(msg.identifier);
    if (axis_id >= len)
    {
        log_msg(&msg, "received message with invalid identifier");
        return true;
    }
    // check message length
    uint8_t cmd_id = ODRIVE_CMD_ID(msg.identifier);
    // Serial.println(axis_id);
    if (msg.data_length_code != 8)
    {
        log_msg(&msg, "received message with invalid length");
        return true;
    }
    // parse message
    switch (cmd_id)
    {
    case ODRIVE_CMD_HEARTBEAT:
    {
        ODriveAxisState old_state = (ODriveAxisState)(axes[axis_id].heartbeat.axis_current_state);
        axes[axis_id].heartbeat = *(ODriveHeartbeat *)msg.data;
        if (axes[axis_id].state_transition_callback &&
            old_state != axes[axis_id].heartbeat.axis_current_state)
        {
            axes[axis_id].state_transition_callback(axis_id,
                                                    (ODriveAxisState)(axes[axis_id].heartbeat.axis_current_state), old_state,
                                                    axes[axis_id].state_transition_context);
        }
        axes[axis_id].updates.heartbeat = true;
        break;
    }
    case ODRIVE_CMD_GET_ENCODER_COUNT:
        axes[axis_id].encoder_count = *(ODriveEncoderCount *)msg.data;
        axes[axis_id].updates.encoder_count = true;
        break;
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
        // log_msg(&msg, "a");
        axes[axis_id].encoder_estimates = *(ODriveEncoderEstimates *)msg.data;
        // printf("Estimate pos %f\n", axes[axis_id].encoder_estimates.position);
        axes[axis_id].updates.encoder_estimates = true;
        break;
    case ODRIVE_CMD_GET_SENSORLESS_ESTIMATES:
        axes[axis_id].sensorless_estimates = *(ODriveSensorlessEstimates *)msg.data;
        axes[axis_id].updates.sensorless_estimates = true;
        break;
    case ODRIVE_CMD_GET_IQ:
        axes[axis_id].iq = *(ODriveIq *)msg.data;
        axes[axis_id].updates.iq = true;
        break;
    case ODRIVE_CMD_GET_VBUS_VOLTAGE:
        axes[axis_id].vbus_voltage = *(float *)msg.data;
        axes[axis_id].updates.vbus_voltage = true;
        break;
    case ODRIVE_CMD_GET_MOTOR_ERROR:
        axes[axis_id].motor_error = *(uint64_t *)msg.data;
        axes[axis_id].updates.motor_error = true;
        break;
    case ODRIVE_CMD_GET_ENCODER_ERROR:
        axes[axis_id].encoder_error = *(uint32_t *)msg.data;
        axes[axis_id].updates.encoder_error = true;
        break;
    case ODRIVE_CMD_GET_SENSORLESS_ERROR:
        axes[axis_id].sensorless_error = *(uint32_t *)msg.data;
        axes[axis_id].updates.sensorless_error = true;
        break;
    case ODRIVE_CMD_GET_CONTROLLER_ERROR:
        axes[axis_id].controller_error = *(uint32_t *)msg.data;
        axes[axis_id].updates.controller_error = true;
        break;
    default:
        log_msg(&msg, "received message with invalid command");
        return true;
    }

    // Serial.printf("id: %d; hb: %d\n", 0, axes[0].updates.heartbeat);
    return error;
}

void set_mode(int id)
{
    ODriveControllerModes mode = {
        ODRIVE_CONTROL_MODE_TORQUE,
        ODRIVE_INPUT_MODE_TORQUE_RAMP,
    };
    odrive_send_command(id, ODRIVE_CMD_SET_CONTROLLER_MODES, &mode, sizeof(mode));
    uint32_t state = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
    if (axes[id].heartbeat.axis_current_state != state)
    {
        printf("Err %d\n", odrive_send_command(id, ODRIVE_CMD_SET_REQUESTED_STATE, &state, sizeof(state)));
    }
}

#endif