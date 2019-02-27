#include "chassis_behaviour.h"
#include "chassis_task.h"

// Watts
uint16_t get_motor_power(const motor_measure_t *motor)
{
    return (motor->given_current) * 24;
}


// Watts
uint16_t get_total_motor_power(chassis_move_t *chassis_move)
{
    uint16_t to_return = 0;
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
				uint16_t motor_power = get_motor_power((chassis_move->motor_chassis[i]).chassis_motor_measure);
				to_return += motor_power;
    }
    return to_return;
}

// TODO serial_send(uint16_t power)
