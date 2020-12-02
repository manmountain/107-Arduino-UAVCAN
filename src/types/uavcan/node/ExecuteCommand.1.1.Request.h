/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_TRANSFER_UAVCAN_NODE_EXECUTE_COMMAND_1_1_REQUEST_H_
#define ARDUINO_TRANSFER_UAVCAN_NODE_EXECUTE_COMMAND_1_1_REQUEST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include "ExecuteCommand.1.1.nnvg.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace ExecuteCommand_1_1
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Request
{

public:

  enum class Command : uint16_t
  {
    RESTART                 = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART,
    POWER_OFF               = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_POWER_OFF,
    BEGIN_SOFTWARE_UPDATE   = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_BEGIN_SOFTWARE_UPDATE,
    FACTORY_RESET           = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_FACTORY_RESET,
    EMERGENCY_STOP          = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_EMERGENCY_STOP,
    STORE_PERSISTENT_STATES = uavcan_node_ExecuteCommand_Request_1_1_COMMAND_STORE_PERSISTENT_STATES,
  };

  uavcan_node_ExecuteCommand_Request_1_1 data;

  static constexpr CanardPortID       PORT_ID = uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_;
  static constexpr size_t             MAX_PAYLOAD_SIZE = uavcan_node_ExecuteCommand_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND = CanardTransferKindRequest;

  Request();
  Request(Request const & other);

  static Request deserialize(CanardTransfer const & transfer);
  size_t serialize(uint8_t * payload) const;

  void operator = (Command const command);  

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* ExecuteCommand_1_1 */

#endif /* ARDUINO_TRANSFER_UAVCAN_NODE_EXECUTE_COMMAND_1_0_REQUEST_H_ */
