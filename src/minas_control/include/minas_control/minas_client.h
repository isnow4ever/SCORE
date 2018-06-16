/****************************************************************************
# minas_client.h:  MINAS A5B EtherCAT Motor Controller                      #
# Copyright (C) 2016, 2017, Tokyo Opensource Robotics Kyokai Association    #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#ifndef MINAS_CLIENT_H
#define MINAS_CLIENT_H

#include <ethercat_manager/ethercat_manager.h>
#include <soem/osal.h>


// Forward declaration of EtherCatManager
namespace ethercat
{
class EtherCatManager;
}

namespace minas_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

// Default PDO maping 4
// Position contorl, Velocity COntrol, Torque Controle (Touch proble, torque limit)
typedef struct {
  // input
  uint16 error_code;			// 603Fh : Error code
  uint16 statusword;			// 6041h : Statusword
  uint8  operation_mode;		// 6061h : Modes of operation display
  uint32 position_actual_value;		// 6064h : Position actual value
  uint32 velocity_actual_value;		// 606Ch : Velocity actual value
  uint16 torque_actual_value;		// 6077h : Torque actual value
  uint16 touch_probe_status;		// 60B9h : Touch probe status
  uint32 touch_probe_posl_pos_value;	// 60BAh : Touch probe pos1 pos value
  uint32 digital_inputs;		// 60FDh : Digital inputs
} MinasInput;

typedef struct {
  uint16 controlword;			// 6040h : Controlword
  uint8  operation_mode;		// 6060h : Mode of operation
  uint16 target_torque;			// 6071h : Target Torque
  uint16 max_torque;			// 6072h : Max Torque
  uint32 target_position;		// 607Ah : Target Position
  uint32 max_motor_speed;		// 6080h : Max motor speed
  uint16 touch_probe_function;		// 60B8h : Touch Probe function
  uint32 target_velocity;		// 60FFh : Target Velocity
  uint32 position_offset;		// 60B0h : Position Offset
  //uint32 digital_outputs;       // 60FEh :  Digital Outputs
} MinasOutput;

typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;  // Statusword(6041h) SX-DSV02470 p.78

typedef enum {NO_MODE_CHANGE, PROFILE_POSITION_MODE, VELOCITY_MODE, PROFILE_VELOCITY_MODE, TORQUE_PROFILE_MODE, HOMING_MODE, INTERPOLATED_POSITION_MODE, CYCLIC_SYNCHRONOUS_POSITION_MODE, CYCLIC_SYNCHRONOUS_VELOCITY_MODE, CYCLIC_SYNCHRONOUS_TORQUE_MODE} PDS_OPERATION; // Mode of operation(6061h) SX-DSV02470 p.82

 typedef enum {//HALT, FAULT_RESET, ENABLE_OPERATION, QUICK_STOP, ENABLE_VOLTAGE, SWITCH_ON, 
} PDS_CONTROL; // Controlworld(6040h) SX-DSV02470 p.76

class MinasClient
{
public:
  /**
   * \brief Constructs a control interface to a MINUS AC Servo on
   *        the given ethercat network and the given slave_no.
   *
   * @param[in] manager The interface to an EtherCAT network that the gripper
   *                    is connected to.
   *
   * @param[in] slave_no The slave number of the gripper on the EtherCAT network
   *                     (>= 1)
   */
  MinasClient(ethercat::EtherCatManager& manager, int slave_no);

  /**
   * \brief Write the given set of control flags to the memory of the controller
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const MinasOutput& output);

  /**
   * \brief Reads set of input-register values from the controller.
   * \return The AC servo input registers as read from the controller IOMap
   */
  MinasInput readInputs() const;

  /**
   * \brief Reads set of output-register values from the controller.
   * \return The AC servo output registers as read from the controller IOMap
   */
  MinasOutput readOutputs() const;

  /**
   * \brief Reset alarm
   * \return void
   */
  void reset();

  /**
   * \brief Send servo on sequence to the controller
   * \return void
   */
  void servoOn();

  /**
   * \brief Send servo off sequence to the controller
   * \return void
   */
  void servoOff();

  /*
   * \brief Torque setup for emergency stop [%] 0 - 500 (3511h / 00h)
   * \return void
   */
  void setTrqueForEmergencyStop(double val);

  /*
   * \brief Over-load level setup [%] 0 - 500 (3512h / 00h)
   * \return void
   */
  void setOverLoadLevel(double val);

  /*
   * \brief Over-speed level setup [r/min] 0 - 2000 (3513h / 00h)
   * \return void
   */
  void setOverSpeedLevel(double val);

  /*
   * \brief Motor working range setup [revolution] 0 - 100 (3514h / 00h)
   * \return void
   */
  void setMotorWorkingRange(double val);

  /*
   * \brief set Profile velocity 0 - 4294967295 (6081h / 00h)
   * \return void
   */
  void setProfileVelocity(uint32_t val);

  /*
   * \brief set Interpolation Time Period 250, 500, 1000, 2000, 4000 us
   * \return void
   */
  void setInterpolationTimePeriod(int us);

  /**
   * \brief print status from input data
   */
  void printPDSStatus(const MinasInput input) const;

  /**
   * \brief print operation mode from input data
   */
  void printPDSOperation(const MinasInput input) const;

  /**
   * \brief print control status from input data
   */
  void printPDSControl(const MinasInput input) const;

private:
  /**
   * \brief get status from input data
   * \return status
   */
  PDS_STATUS getPDSStatus(const MinasInput input) const;

  /**
   * \brief get operation mode from input data
   * \return status
   */
  PDS_OPERATION getPDSOperation(const MinasInput input) const;

  /**
   * \brief get control status from input data
   * \return status
   */
  PDS_STATUS getPDSControl(const MinasInput input) const;

  ethercat::EtherCatManager& manager_;
  const int slave_no_;
};

/**
 * \brief Table of error code and text
 * See 6-3 of https://industrial.panasonic.com/content/data/MT/PDF/manual/en/acs/minas-a5-2_manu_e.pdf
 */
const struct {
  unsigned int code;
  const char* text;
} error_map[] = {
  {11, "Control power supply under-voltage protection"},
  {12, "Over voltage protection"},
  {13, "Main power supply under-voltage protection(between P and N)"},
  {14, "Over-current protection"}, 
  {15, "Over-heat protection"},
  {16, "Over-load protection"},
  {18, "Over-regeneration load protection"},
  {21, "Encoder communication disconnect error protection"},
  {23, "Encoder communication data error protection"},
  {24, "Position deviation excess protection"},
  {25, "Hybrid deviation excess error protection"},
  {26, "Over speed protection"},
  {27, "Command pulse input frequency error protection"},
  {28, "Limit of pulse replay error protection"},
  {29, "Deviation counter overflow protection"},
  {30, "Safety detection"},
  {33, "IF overlaps allocation error 1 protection"},
  {34, "Software limit protection"},
  {36, "EEPROM parameter error protection"},
  {37, "EEPROM check code error protection"},
  {38, "Over-travel inhibit input protection"},
  {39, "Analog input1 excess protection"},
  {40, "Absolute system down error protection"},
  {41, "Absolute counter over error protection"},
  {42, "Absolute over-speed error protection"},
  {43, "Initialization failure"},
  {44, "Absolute single turn counter error protection"},
  {45, "Absolute multi-turn counter error protection"},
  {47, "Absolute status error protection"},
  {48, "Encoder Z-phase error protection"},
  {49, "Encoder CS signal error protection"},
  {50, "Feedback scale connection error protection"},
  {51, "Feedback scale status 0 error protection"},
  {55, "A-phase connection error protection"},
  {87, "Compulsory alarm input protection"},
  {95, "Motor automatic recognition error protection"},
  {99, "Other error"},
};

}

#endif
