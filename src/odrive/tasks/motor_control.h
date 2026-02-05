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

        bool errors = false;
        bool errors_read = true;
        for (int i = 0; i < 2; i++)
        {
            if (axes[i].heartbeat.axis_error) {
                errors = true;
            }
            if (axes[i].heartbeat.motor_error_flag) {
                odrive_send_get_command(i, ODRIVE_CMD_GET_MOTOR_ERROR);
                errors = true;
                if (!axes[i].motor_error)
                    errors_read = false;
            }
            if (axes[i].heartbeat.encoder_error_flag) {
                odrive_send_get_command(i, ODRIVE_CMD_GET_ENCODER_ERROR);
                errors = true;
                if (!axes[i].motor_error)
                    errors_read = false;
            }
            if (axes[i].heartbeat.controller_error_flag) {
                odrive_send_get_command(i, ODRIVE_CMD_GET_CONTROLLER_ERROR);
                errors = true;
                if (!axes[i].motor_error)
                    errors_read = false;
            }
        }
        if (!errors) {
            for (int i = 0; i < 2; i++)
            {
                axes[i].last_errors = { 0 };
            }
        }
        if (errors && errors_read) {
            for (int i = 0; i < 2; i++)
            {
                axes[i].last_errors.axis_error = axes[i].heartbeat.axis_error;
                axes[i].last_errors.controller_error = axes[i].controller_error;
                axes[i].last_errors.encoder_error = axes[i].encoder_error;
                axes[i].last_errors.motor_error = axes[i].motor_error;
                axes[i].last_errors.sensorless_error = axes[i].sensorless_error;
                odrive_send_get_command(i, ODRIVE_CMD_CLEAR_ERRORS);
            }   
        }

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