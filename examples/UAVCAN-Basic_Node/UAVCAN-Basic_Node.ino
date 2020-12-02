/*
 * This example shows how to use the UAVCAN library to set up a basic node
 * responding to requests from service clients. 
 *
 * Hardware:
 *   - Arduino MKR family board, e.g. MKR VIDOR 4000
 *   - Arduino MKR CAN shield
 *
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <SPI.h>
#include <ArduinoMCP2515.h>
#include <ArduinoUAVCAN.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/


/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/
static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;
static uint8_t const node_id = 0x301;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/
void    spi_select      ();
void    spi_deselect    ();
uint8_t spi_transfer    (uint8_t const);
void    onExternalEvent ();
void    onReceiveBufferFull(CanardFrame const &);
bool    transmitCanFrame(CanardFrame const &);
void onExecuteCommand_1_0_Request_Received(CanardTransfer const &, ArduinoUAVCAN &);
void onGetInfo_1_1_Request_Received(CanardTransfer const &, ArduinoUAVCAN &);
void(* resetArduinoFunc) (void) = 0; //declare reset function @ address 0

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/
bool delayedReboot = false;
ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);
ArduinoUAVCAN uavcan(node_id, transmitCanFrame);
Heartbeat_1_0 heartbeat;
GetInfo_1_0::Response getInfoRespons;
/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/
void setup()
{
  Serial.begin(9600);
  while(!Serial) { }

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  heartbeat.data.uptime = 0;
  heartbeat = Heartbeat_1_0::Health::NOMINAL;
  heartbeat = Heartbeat_1_0::Mode::INITIALIZATION;
  heartbeat.data.vendor_specific_status_code = 0;

  /* Configure GetInfo response parameters */
  uint8_t const unique_id[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  Version_1_0 protocol_version = Version_1_0(1, 0);
  Version_1_0 hardware_version = Version_1_0(0, 1);
  Version_1_0 software_version = Version_1_0(1, 2);
  uint8_t node_name[] = "cc.arduino.node.example";
  // set parameters to GetInfo response
  getInfoRespons.data.protocol_version = protocol_version.data;
  getInfoRespons.data.hardware_version = hardware_version.data;
  getInfoRespons.data.software_version = software_version.data;
  getInfoRespons.data.software_vcs_revision_id = 0x1234;
  std::copy(std::begin(unique_id), std::end(unique_id),  std::begin(getInfoRespons.data.unique_id));  
  std::copy(std::begin(node_name), std::end(node_name),  std::begin(getInfoRespons.data.name.elements));
  getInfoRespons.data.name.count = std::end(node_name) - std::begin(node_name);
  //getInfoRespons.data.software_image_crc.elements
  //getInfoRespons.data.software_image_crc.count
  //getInfoRespons.data.certificate_of_authenticity.elements
  //getInfoRespons.data.certificate_of_authenticity.count
  
  /* Subscribe to incoming get info requests */
  uavcan.subscribe<GetInfo_1_0::Request>(onGetInfo_1_0_Request_Received);
  /* Subscribe to incoming service requests */
  uavcan.subscribe<ExecuteCommand_1_0::Request>(onExecuteCommand_1_1_Request_Received);
}

void loop()
{
  /* Update the heartbeat object */
  heartbeat.data.uptime = millis() / 1000;
  heartbeat = Heartbeat_1_0::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
    /* send heartbeat */
    uavcan.publish(heartbeat);
    prev = now;
  }
  
  /* Transmit all enqeued CAN frames */
  while(uavcan.transmitCanFrame()) { }
  // are we in a delayed reboot state?
  if (delayedReboot) {
    resetArduinoFunc();  //call reset too reboot arduino
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/
void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  uavcan.onCanFrameReceived(frame);
}

bool transmitCanFrame(CanardFrame const & frame)
{
  return mcp2515.transmit(frame);
}
/**************************************************************************************
 * SERVICE REQUEST HANDLERS
 **************************************************************************************/
void onExecuteCommand_1_1_Request_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uavcan)
{
  ExecuteCommand_1_1::Request  req = ExecuteCommand_1_1::Request::deserialize(transfer);
  ExecuteCommand_1_1::Response rsp;
  
  // Built in commands..
  switch(req.data.command)
  {
    case (uint16_t)ExecuteCommand_1_1::Request::Command::RESTART:
    case (uint16_t)ExecuteCommand_1_1::Request::Command::POWER_OFF:
      Serial.println("Reboot in progress..");
      rsp = ExecuteCommand_1_1::Response::Status::SUCCESS;
      uavcan.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
      delayedReboot = true;
      break;
    case (uint16_t)ExecuteCommand_1_1::Request::Command::BEGIN_SOFTWARE_UPDATE:
    case (uint16_t)ExecuteCommand_1_1::Request::Command::FACTORY_RESET:
    case (uint16_t)ExecuteCommand_1_1::Request::Command::EMERGENCY_STOP:
    case (uint16_t)ExecuteCommand_1_1::Request::Command::STORE_PERSISTENT_STATES:
      Serial.println("TODO! implement the rest of the built in commands!!!");
      rsp = ExecuteCommand_1_1::Response::Status::INTERNAL_ERROR;
      uavcan.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
      break;
    default:
      // Custom commands
      if (req.data.command == 0xCAFE) {
        Serial.print("Command 0xCAFE with parameter: ");
        Serial.write(req.data.parameter.elements, req.data.parameter.count);
        Serial.println(" received");
        ExecuteCommand_1_1::Response rsp;
        rsp = ExecuteCommand_1_1::Response::Status::SUCCESS;
        uavcan.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
      } else {
        ExecuteCommand_1_1::Response rsp;
        rsp = ExecuteCommand_1_1::Response::Status::NOT_AUTHORIZED;
        uavcan.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
      }
  }  
}

void onGetInfo_1_0_Request_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uavcan)
{
  GetInfo_1_0::Request req = GetInfo_1_0::Request::deserialize(transfer);
  /* the getInfo respons should NEVER change while the node is running. Send the response configured at boot-up */
  uavcan.respond(getInfoRespons, transfer.remote_node_id, transfer.transfer_id);
}
