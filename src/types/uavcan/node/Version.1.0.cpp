/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "Version.1.0.h"

#include "../../../utility/convert.hpp"

/**************************************************************************************
 * STATIC CONSTEXPR DEFINITION
 **************************************************************************************/

//constexpr CanardPortID       Version::PORT_ID;
constexpr size_t             Version_1_0::MAX_PAYLOAD_SIZE;
constexpr CanardTransferKind Version_1_0::TRANSFER_KIND;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Version_1_0::Version_1_0()
{
  uavcan_node_Version_1_0_initialize_(&data);
}

Version_1_0::Version_1_0(uint8_t major, uint8_t minor)
{
  uavcan_node_Version_1_0_initialize_(&data);
  data.major = major;
  data.minor = minor;
}

Version_1_0::Version_1_0(Version_1_0 const & other)
{
  memcpy(&data, &other.data, sizeof(data));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

Version_1_0 Version_1_0::deserialize(CanardTransfer const & transfer)
{
  Version_1_0 h;
  size_t inout_buffer_size_bytes = transfer.payload_size;
  uavcan_node_Version_1_0_deserialize_(&h.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
  return h;
}

size_t Version_1_0::serialize(uint8_t * payload) const
{
  size_t inout_buffer_size_bytes = Version_1_0::MAX_PAYLOAD_SIZE;

  if (uavcan_node_Version_1_0_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
    return 0;
  else
    return inout_buffer_size_bytes;
}

/*void Version_1_0::operator = (Version_1_0 const version)
{
  data.major = version.data.major;
  data.minor = version.data.minor;
}*/

uavcan_node_Version_1_0 Version_1_0::operator = (Version_1_0 const version)
{
  return data;
}

