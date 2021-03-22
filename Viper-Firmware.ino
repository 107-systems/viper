/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/UAVCAN-GNSS-node/graphs/contributors
 */

/* Recommended hardware setup:
 *  MKR Zero <-> MKR CAN Shield <-> MKR GPS Shield
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>
#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <ArduinoDebug.hpp>

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const UAVCAN_NODE_ID         = 13;
static int     const MKRCAN_MCP2515_CS_PIN  = 3;
static int     const MKRCAN_MCP2515_INT_PIN = 7;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/
 
uint8_t spi_transfer(uint8_t const);

namespace MCP2515
{
void select();
void deselect();
void onExternalEvent();
void onReceive(CanardFrame const &);
bool transmit(CanardFrame const &);
}

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(MCP2515::select,
                       MCP2515::deselect,
                       spi_transfer,
                       micros,
                       MCP2515::onReceive,
                       nullptr);

ArduinoUAVCAN uavcan_hdl(UAVCAN_NODE_ID, MCP2515::transmit);

DEBUG_INSTANCE(120, Serial);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  /* USB serial for debug messages.
   */
  Serial.begin(115200);
  while(!Serial) { }

  /* Serial connected to MKR GPS board.
   */
  Serial1.begin(9600);

  /* Setup SPI access
   */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register 
   * MCP2515 signaled by taking INT low.
   */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), MCP2515::onExternalEvent, FALLING);

  /* Configure MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setListenOnlyMode();
}

void loop()
{
  /* Transmit all enqeued CAN frames */
  while(uavcan_hdl.transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

/**************************************************************************************
 * FUNCTION DEFINITION MCP2515
 **************************************************************************************/

namespace MCP2515
{

void select() {
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void deselect() {
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

void onExternalEvent() {
  mcp2515.onExternalEventHandler();
}

void onReceive(CanardFrame const & frame) {
  uavcan_hdl.onCanFrameReceived(frame);
}

bool transmit(CanardFrame const & frame) {
  return mcp2515.transmit(frame);
}

} /* MCP2515 */
