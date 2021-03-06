/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_TRANSFER_UAVCAN_NODE_Version_1_0_H_
#define ARDUINO_TRANSFER_UAVCAN_NODE_Version_1_0_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include "Version.1.0.nnvg.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Version_1_0
{

public:

  uavcan_node_Version_1_0 data;

  //static constexpr CanardPortID       PORT_ID = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = uavcan_node_Version_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND = CanardTransferKindMessage;


  Version_1_0();
  Version_1_0(uint8_t minor, uint8_t major);
  Version_1_0(Version_1_0 const & other);

  static Version_1_0 deserialize(CanardTransfer const & transfer);
  size_t serialize(uint8_t * payload) const;

  //void operator = (Version_1_0 const version);
  uavcan_node_Version_1_0 operator = (Version_1_0 const version);
};

#endif /* ARDUINO_TRANSFER_UAVCAN_NODE_Version_1_0_H_ */
