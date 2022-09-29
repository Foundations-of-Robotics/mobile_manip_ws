/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/message.h"
#include "puma_motor_msgs/Status.h"

#include <string>
#include <ros/ros.h>

namespace puma_motor_driver
{

namespace ConfigurationStates
{
  enum ConfigurationState
  {
    Unknown = -1,
    Initializing,
    PowerFlag,
    EncoderPosRef,
    EncoderSpdRef,
    EncoderCounts,
    ClosedLoop,
    ControlMode,
    PGain,
    IGain,
    DGain,
    VerifiedParameters,
    Configured
  };
}  // namespace ConfigurationStates
typedef ConfigurationStates::ConfigurationState ConfigurationState;

Driver::Driver(Gateway& gateway, const uint8_t& device_number, const std::string& device_name)
  : gateway_(gateway), device_number_(device_number), device_name_(device_name),
    configured_(false), state_(ConfigurationState::Initializing), control_mode_(puma_motor_msgs::Status::MODE_SPEED),
    gain_p_(1), gain_i_(0), gain_d_(0), encoder_cpr_(1), gear_ratio_(1)
  {
  }

void Driver::processMessage(const Message& received_msg)
{
  // If it's not our message, jump out.
  if (received_msg.getDeviceNumber() != device_number_) return;

  // If there's no data then this is a request message, jump out.
  if (received_msg.len == 0) return;

  Field* field = nullptr;
  if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_CFG) == CAN_API_MC_CFG)
  {
    field = cfgFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_STATUS) == CAN_API_MC_STATUS)
  {
    field = statusFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_ICTRL) == CAN_API_MC_ICTRL)
  {
    field = ictrlFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_POS) == CAN_API_MC_POS)
  {
    field = posFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_VCOMP) == CAN_API_MC_VCOMP)
  {
    field = vcompFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_SPD) == CAN_API_MC_SPD)
  {
    field = spdFieldForMessage(received_msg);
  }
  else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_VOLTAGE) == CAN_API_MC_VOLTAGE)
  {
    field = voltageFieldForMessage(received_msg);
  }

  if (!field) return;

  // Copy the received data and mark that field as received.
  std::memcpy(field->data, received_msg.data, received_msg.len);
  field->received = true;
}

double Driver::radPerSecToRpm() const
{
  return ((60 * gear_ratio_) / (2 * M_PI));
}

void Driver::sendUint8(const uint32_t id, const uint8_t value)
{
  Message msg;
  msg.id = id;
  msg.len = 1;
  std::memcpy(msg.data, &value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendUint16(const uint32_t id, const uint16_t value)
{
  Message msg;
  msg.id = id;
  msg.len = 2;
  std::memcpy(msg.data, &value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendFixed8x8(const uint32_t id, const float value)
{
  Message msg;
  msg.id = id;
  msg.len = 2;
  int16_t output_value = static_cast<int16_t>(static_cast<float>(1<<8) * value);
  std::memcpy(msg.data, &output_value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendFixed16x16(const uint32_t id, const double value)
{
  Message msg;
  msg.id = id;
  msg.len = 4;
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1<<16) * value));
  std::memcpy(msg.data, &output_value, msg.len);
  gateway_.queue(msg);
}

bool Driver::verifyRaw16x16(const uint8_t* received, const double expected)
{
  uint8_t data[4];
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1<<16) * expected));
  std::memcpy(data, &output_value, 4);
  for (uint8_t i = 0; i < 4; i++)
  {
    if (*received != data[i])
    {
      return false;
    }
    received++;
  }
  return true;
}

bool Driver::verifyRaw8x8(const uint8_t* received, const float expected)
{
  uint8_t data[2];
  int32_t output_value = static_cast<int32_t>(static_cast<float>((1<<8) * expected));
  std::memcpy(data, &output_value, 2);
  for (uint8_t i = 0; i < 2; i++)
  {
    if (*received != data[i])
    {
      return false;
    }
    received++;
  }
  return true;
}

void Driver::setEncoderCPR(const uint16_t encoder_cpr)
{
  encoder_cpr_ = encoder_cpr;
}

void Driver::setGearRatio(const float gear_ratio)
{
  gear_ratio_ = gear_ratio;
}

void Driver::commandDutyCycle(const float cmd)
{
  sendFixed8x8((LM_API_VOLT_SET | device_number_), cmd);
}

void Driver::commandSpeed(const double cmd)
{
  // Converting from rad/s to RPM through the gearbox.
  sendFixed16x16((LM_API_SPD_SET | device_number_), (cmd * radPerSecToRpm()));
}

void Driver::verifyParams()
{
  switch (state_)
  {
    case ConfigurationState::Initializing:
      ROS_INFO("Puma Motor Controller on %s (%i): starting to verify parameters.",
          device_name_.c_str(), device_number_);
      state_ = ConfigurationState::PowerFlag;
      break;
    case ConfigurationState::PowerFlag:
      if (lastPower() == 0)
      {
        state_ = ConfigurationState::EncoderPosRef;
        ROS_INFO("Puma Motor Controller on %s (%i): cleared power flag.", device_name_.c_str(), device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
      }
      break;
    case ConfigurationState::EncoderPosRef:
      if (posEncoderRef() == LM_REF_ENCODER)
      {
        state_ = ConfigurationState::EncoderSpdRef;
        ROS_INFO("Puma Motor Controller on %s (%i): set position encoder reference.",
            device_name_.c_str(), device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_POS_REF | device_number_));
      }
      break;
    case ConfigurationState::EncoderSpdRef:
      if (spdEncoderRef() == LM_REF_QUAD_ENCODER)
      {
        state_ = ConfigurationState::EncoderCounts;
        ROS_INFO("Puma Motor Controller on %s (%i): set speed encoder reference.",
            device_name_.c_str(), device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_SPD_REF | device_number_));
      }
      break;
    case ConfigurationState::EncoderCounts:
      if (encoderCounts() == encoder_cpr_)
      {
        state_ = ConfigurationState::ClosedLoop;
        ROS_INFO("Puma Motor Controller on %s (%i): set encoder counts to %i.",
            device_name_.c_str(), device_number_, encoder_cpr_);
      }
      else
      {
        gateway_.queue(Message(LM_API_CFG_ENC_LINES | device_number_));
      }
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      if (lastMode() == puma_motor_msgs::Status::MODE_SPEED)
      {
        state_ = ConfigurationState::ControlMode;
        ROS_INFO("Puma Motor Controller on %s (%i): entered a close-loop control mode.",
            device_name_.c_str(), device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_STATUS_CMODE | device_number_));
      }
      break;
    case ConfigurationState::ControlMode:
      if (lastMode() == control_mode_)
      {
        if (control_mode_ != puma_motor_msgs::Status::MODE_VOLTAGE)
        {
          state_ = ConfigurationState::PGain;
          ROS_INFO("Puma Motor Controller on %s (%i): was set to a close loop control mode.",
              device_name_.c_str(), device_number_);
        }
        else
        {
          state_ = ConfigurationState::VerifiedParameters;
          ROS_INFO("Puma Motor Controller on %s (%i): was set to voltage control mode.",
              device_name_.c_str(), device_number_);
        }
      }
      break;
    case ConfigurationState::PGain:
      if (verifyRaw16x16(getRawP(), gain_p_))
      {
        state_ = ConfigurationState::IGain;
        ROS_INFO("Puma Motor Controller on %s (%i): P gain constant was set to %f and %f was requested.",
            device_name_.c_str(), device_number_, getP(), gain_p_);
      }
      else
      {
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_PC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_PC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_PC | device_number_));
            break;
        }
      }
      break;
    case ConfigurationState::IGain:
      if (verifyRaw16x16(getRawI(), gain_i_))
      {
        state_ = ConfigurationState::DGain;
        ROS_INFO("Puma Motor Controller on %s (%i): I gain constant was set to %f and %f was requested.",
            device_name_.c_str(), device_number_, getI(), gain_i_);
      }
      else
      {
        ROS_WARN("Puma Motor Controller on %s (%i): I gain constant was set to %f and %f was requested.",
           device_name_.c_str(), device_number_, getI(), gain_i_);
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_IC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_IC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_IC | device_number_));
            break;
        }
      }
      break;
    case ConfigurationState::DGain:
      if (verifyRaw16x16(getRawD(), gain_d_))
      {
        state_ = ConfigurationState::VerifiedParameters;
        ROS_INFO("Puma Motor Controller on %s (%i): D gain constant was set to %f and %f was requested.",
            device_name_.c_str(), device_number_, getD(), gain_d_);
      }
      else
      {
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_DC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_DC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_DC | device_number_));
            break;
        }
      }
      break;
  }
  if (state_ == ConfigurationState::VerifiedParameters)
  {
    ROS_INFO("Puma Motor Controller on %s (%i): all parameters verified.", device_name_.c_str(), device_number_);
    configured_ = true;
    state_ = ConfigurationState::Configured;
  }
}

void Driver::configureParams()
{
  switch (state_)
  {
    case ConfigurationState::PowerFlag:
      sendUint8((LM_API_STATUS_POWER | device_number_), 1);
      break;
    case ConfigurationState::EncoderPosRef:
      sendUint8((LM_API_POS_REF | device_number_), LM_REF_ENCODER);
      break;
    case ConfigurationState::EncoderSpdRef:
      sendUint8((LM_API_SPD_REF | device_number_), LM_REF_QUAD_ENCODER);
      break;
    case ConfigurationState::EncoderCounts:
      // Set encoder CPR
      sendUint16((LM_API_CFG_ENC_LINES | device_number_), encoder_cpr_);
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      gateway_.queue(Message(LM_API_SPD_EN | device_number_));
      break;
    case ConfigurationState::ControlMode:
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_VOLTAGE:
          gateway_.queue(Message(LM_API_VOLT_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_CURRENT:
          gateway_.queue(Message(LM_API_ICTRL_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          gateway_.queue(Message(LM_API_POS_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          gateway_.queue(Message(LM_API_SPD_EN | device_number_));
          break;
      }
      break;
    case ConfigurationState::PGain:
      // Set P
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_PC  | device_number_), gain_p_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_PC  | device_number_), gain_p_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_PC  | device_number_), gain_p_);
          break;
      }
      break;
    case ConfigurationState::IGain:
      // Set I
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_IC  | device_number_), gain_i_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_IC  | device_number_), gain_i_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_IC  | device_number_), gain_i_);
          break;
      }
      break;
    case ConfigurationState::DGain:
      // Set D
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_DC  | device_number_), gain_d_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_DC  | device_number_), gain_d_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_DC  | device_number_), gain_d_);
          break;
      }
      break;
  }
}

bool Driver::isConfigured() const
{
  return configured_;
}

void Driver::setGains(const double p, const double i, const double d)
{
  gain_p_ = p;
  gain_i_ = i;
  gain_d_ = d;

  if (configured_)
  {
    updateGains();
  }
}

void Driver::setMode(const uint8_t mode)
{
  if (mode == puma_motor_msgs::Status::MODE_VOLTAGE)
  {
    control_mode_ = mode;
    ROS_INFO("Puma Motor Controller on %s (%i): mode set to voltage control.", device_name_.c_str(), device_number_);
    if (configured_)
    {
      resetConfiguration();
    }
  }
  else
  {
    ROS_ERROR("Puma Motor Controller on %s (%i): Close loop modes need PID gains.",
        device_name_.c_str(), device_number_);
  }
}

void Driver::setMode(const uint8_t mode, const double p, const double i, const double d)
{
  if (mode == puma_motor_msgs::Status::MODE_VOLTAGE)
  {
    control_mode_ = mode;
    ROS_WARN("Puma Motor Controller on %s (%i): mode set to voltage control but PID gains are not needed.",
        device_name_.c_str(), device_number_);
    if (configured_)
    {
      resetConfiguration();
    }
  }
  else
  {
    control_mode_ = mode;
    if (configured_)
    {
      resetConfiguration();
    }
    setGains(p, i, d);
    ROS_INFO(
        "Puma Motor Controller on %s (%i): mode set to a closed-loop control with PID gains of P:%f, I:%f and D:%f.",
        device_name_.c_str(), device_number_, gain_p_, gain_i_, gain_d_);
  }
}

void Driver::clearMsgCache()
{
  // Set it all to zero, which will in part clear
  // the boolean flags to be false.
  memset(voltage_fields_, 0, sizeof(voltage_fields_));
  memset(spd_fields_, 0, sizeof(spd_fields_));
  memset(vcomp_fields_, 0, sizeof(vcomp_fields_));
  memset(pos_fields_, 0, sizeof(pos_fields_));
  memset(ictrl_fields_, 0, sizeof(ictrl_fields_));
  memset(status_fields_, 0, sizeof(status_fields_));
  memset(cfg_fields_, 0, sizeof(cfg_fields_));
}

void Driver::requestStatusMessages()
{
  gateway_.queue(Message(LM_API_STATUS_POWER   | device_number_));
}

void Driver::requestFeedbackMessages()
{
  gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_POS     | device_number_));
  gateway_.queue(Message(LM_API_STATUS_SPD     | device_number_));
  gateway_.queue(Message(LM_API_SPD_SET        | device_number_));
}
void Driver::requestFeedbackDutyCycle()
{
  gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
}

void Driver::requestFeedbackCurrent()
{
  gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
}

void Driver::requestFeedbackPosition()
{
  gateway_.queue(Message(LM_API_STATUS_POS | device_number_));
}

void Driver::requestFeedbackSpeed()
{
  gateway_.queue(Message(LM_API_STATUS_SPD | device_number_));
}

void Driver::requestFeedbackPowerState()
{
  gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
}

void Driver::requestFeedbackSetpoint()
{
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      gateway_.queue(Message(LM_API_ICTRL_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      gateway_.queue(Message(LM_API_POS_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      gateway_.queue(Message(LM_API_SPD_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      gateway_.queue(Message(LM_API_VOLT_SET | device_number_));
      break;
  };
}

void Driver::resetConfiguration()
{
  configured_ = false;
  state_ = ConfigurationState::Initializing;
}

void Driver::updateGains()
{
  configured_ = false;
  state_ = ConfigurationState::PGain;
}

float Driver::lastDutyCycle()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTOUT));
  return (field->interpretFixed8x8() / 128.0);
}

float Driver::lastBusVoltage()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTBUS));
  return field->interpretFixed8x8();
}

float Driver::lastCurrent()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_CURRENT));
  return field->interpretFixed8x8();
}

double Driver::lastPosition()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_POS));
  return (field->interpretFixed16x16() * ((2 * M_PI) / gear_ratio_));  // Convert rev to rad
}

double Driver::lastSpeed()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_SPD));
  return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60)));  // Convert RPM to rad/s
}

uint8_t Driver::lastFault()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_FAULT));
  return field->data[0];
}

uint8_t Driver::lastPower()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_POWER));
  return field->data[0];
}

uint8_t Driver::lastMode()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_CMODE));
  return field->data[0];
}

float Driver::lastOutVoltage()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_VOUT));
  return field->interpretFixed8x8();
}

float Driver::lastTemperature()
{
  Field* field = statusFieldForMessage(Message(LM_API_STATUS_TEMP));
  return field->interpretFixed8x8();
}

float Driver::lastAnalogInput()
{
  Field* field = statusFieldForMessage(Message(CPR_API_STATUS_ANALOG));
  return field->interpretFixed8x8();
}

double Driver::lastSetpoint()
{
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      return statusCurrentGet();
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      return statusPositionGet();
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      return statusSpeedGet();
      break;
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      return statusDutyCycleGet();
      break;
    default:
      return 0;
      break;
  }
}
double Driver::statusSpeedGet()
{
  Field* field = spdFieldForMessage(Message(LM_API_SPD_SET));
  return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60)));  // Convert RPM to rad/s
}

float Driver::statusDutyCycleGet()
{
  Field* field = voltageFieldForMessage(Message(LM_API_VOLT_SET));
  return (field->interpretFixed8x8() / 128.0);
}

float Driver::statusCurrentGet()
{
  Field* field = ictrlFieldForMessage(Message(LM_API_ICTRL_SET));
  return field->interpretFixed8x8();
}
double Driver::statusPositionGet()
{
  Field* field = posFieldForMessage(Message(LM_API_POS_SET));
  return (field->interpretFixed16x16() * (( 2 * M_PI) / gear_ratio_));  // Convert rev to rad
}

uint8_t Driver::posEncoderRef()
{
  Field* field = posFieldForMessage(Message(LM_API_POS_REF));
  return field->data[0];
}

uint8_t Driver::spdEncoderRef()
{
  Field* field = spdFieldForMessage(Message(LM_API_SPD_REF));
  return field->data[0];
}

uint16_t Driver::encoderCounts()
{
  Field* field = cfgFieldForMessage(Message(LM_API_CFG_ENC_LINES));
  return (static_cast<uint16_t>(field->data[0]) | static_cast<uint16_t>(field->data[1] << 8));
}

double Driver::getP()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_PC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_PC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_PC));
      break;
  };
  return field->interpretFixed16x16();
}

double Driver::getI()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_IC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_IC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_IC));
      break;
  };
  return field->interpretFixed16x16();
}

double Driver::getD()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_DC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_DC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_DC));
      break;
  };
  return field->interpretFixed16x16();
}

uint8_t* Driver::getRawP()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_PC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_PC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_PC));
      break;
  };
  return field->data;
}

uint8_t* Driver::getRawI()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_IC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_IC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_IC));
      break;
  };
  return field->data;
}

uint8_t* Driver::getRawD()
{
  Field* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = ictrlFieldForMessage(Message(LM_API_ICTRL_DC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = posFieldForMessage(Message(LM_API_POS_DC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = spdFieldForMessage(Message(LM_API_SPD_DC));
      break;
  };
  return field->data;
}

Driver::Field* Driver::voltageFieldForMessage(const Message& msg)
{
  uint32_t voltage_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &voltage_fields_[voltage_field_index];
}

Driver::Field* Driver::spdFieldForMessage(const Message& msg)
{
  uint32_t spd_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &spd_fields_[spd_field_index];
}

Driver::Field* Driver::vcompFieldForMessage(const Message& msg)
{
  uint32_t vcomp_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &vcomp_fields_[vcomp_field_index];
}

Driver::Field* Driver::posFieldForMessage(const Message& msg)
{
  uint32_t pos_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &pos_fields_[pos_field_index];
}

Driver::Field* Driver::ictrlFieldForMessage(const Message& msg)
{
  uint32_t ictrl_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &ictrl_fields_[ictrl_field_index];
}

Driver::Field* Driver::statusFieldForMessage(const Message& msg)
{
  uint32_t status_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &status_fields_[status_field_index];
}

Driver::Field* Driver::cfgFieldForMessage(const Message& msg)
{
  uint32_t cfg_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &cfg_fields_[cfg_field_index];
}

}  // namespace puma_motor_driver
