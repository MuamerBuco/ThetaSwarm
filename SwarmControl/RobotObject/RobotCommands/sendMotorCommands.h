#include "Client/udp_client_server.h"

using namespace udp_client_server;

void SendMotorCommands(uint8_t *speeds_and_directions, udp_client *client_object, uint16_t ms_delay);
void SendMotorCommandsForMs(uint8_t *speeds_and_directions, uint16_t time_in_ms, uint16_t send_increment_ms, udp_client* client_object);
void SendMotorShutdownCommand(udp_client* client_object);