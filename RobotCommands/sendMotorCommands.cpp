#include "sendMotorCommands.h"
#include "../CommonFunctions/common.h"

// stream motor commands using the passed udp client, 
// in format [parseReference,  UL-speed, UL-direction, UR-speed, UR-direction, DL-speed, DL-direction, DR-speed, DR-direction]
void SendMotorCommands(uint8_t *speeds_and_directions, udp_client *client_object, uint16_t ms_delay)
{
    client_object->send_bytes(speeds_and_directions, sizeof(speeds_and_directions) + 1);
    msDelay(ms_delay);
}

// send set motor commands for [time_in_ms] milliseconds every [send_increment_ms] using passed udp client
void SendMotorCommandsForMs(uint8_t *speeds_and_directions, uint16_t time_in_ms, uint16_t send_increment_ms, udp_client* client_object)
{
    uint16_t number_of_cycles = time_in_ms/send_increment_ms;

    for(uint16_t i = 0; i < number_of_cycles; i++) {
        SendMotorCommands(speeds_and_directions, client_object, send_increment_ms);
    }
}

void SendMotorShutdownCommand(udp_client* client_object)
{
    uint8_t kill_motors[9] = {1, 0, 0, 0, 0, 0, 0, 0, 0};

    for(uint8_t i = 0; i < 10; i++) {
        SendMotorCommands(kill_motors, client_object, 1);
    }
}