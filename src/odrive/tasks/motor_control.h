#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "globals.h"
#include "config.h"
#include "odrive/utils/odrive_can.h"
#include "odrive/utils/exo.h"

static void odrive_state_transition_callback(
    uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void *context)
{
    Serial.printf("axis_id %u, new_state %u, old_state %u\n", axis_id, new_state, old_state);
}

void taskMotorControl(void *param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UPDATE_PERIOD_MS);

    wait_til_heartbeat(0);
    wait_til_heartbeat(1);

    set_mode(1);
    set_mode(0);
    
    exo_init(1000 / UPDATE_PERIOD_MS);
    
    // Main logic loop
    while (1)
    {
        // odrive_send_get_command(0, ODRIVE_CMD_GET_ENCODER_ESTIMATES);
        // odrive_send_get_command(1, ODRIVE_CMD_GET_ENCODER_ESTIMATES);

        float angle_R = -MOTOR_ANGLE_TO_RAD(axes[0].encoder_estimates.position);
        float angle_L = MOTOR_ANGLE_TO_RAD(axes[1].encoder_estimates.position);

        float iq = -exo_update(angle_L, angle_R, delay_setting, gain_setting);

        odrive_send_command(1, ODRIVE_CMD_SET_INPUT_TORQUE, &iq, sizeof(iq));
        odrive_send_command(0, ODRIVE_CMD_SET_INPUT_TORQUE, &iq, sizeof(iq));
        // printf("Motor state: %d\n", axes[0].heartbeat.axis_current_state);
        vTaskDelayUntil(&last_wake, period);
    }
}

#endif