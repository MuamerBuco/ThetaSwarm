#include "udp_client_server.h"
#include <memory>

void SendRobotCommands(uint8_t *speeds_and_directions, std::shared_ptr<udp_client_server::udp_client> client_object, uint16_t ms_delay);
void SendRobotCommandsForMs(uint8_t *speeds_and_directions, uint16_t time_in_ms, uint16_t send_increment_ms, std::shared_ptr<udp_client_server::udp_client> client_object);
// void SendMotorShutdownCommand(udp_client* client_object);