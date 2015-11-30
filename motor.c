/*
 * motor.c
 *
 * Implementation file for operation of L6470 motor controller on TI Stellaris LM4F120H5QR
 *  Created on: Apr 17, 2013
 *      Author: Eric ***REMOVED***, Mateus ***REMOVED***
 *
 *      Using dSPIN example code - 12/12/2011- Mike Hord, SparkFun Electronics
 */

#include"motor.h"


#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

//------------------------------------------------------------------------------------------
//   BEGIN COMMAND CODE ADAPTED FROM dSPIN_command.ino
//   dSPIN_commands.ino - Contains high-level command implementations- movement
//   and configuration commands, for example.
//------------------------------------------------------------------------------------------

// MOTOR A

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void dSPINA_SetParam(unsigned char param, unsigned long value){
  dSPINA_Xfer(dSPIN_SET_PARAM | param);
  dSPINA_ParamHandler(param, value);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
unsigned long dSPINA_GetParam(unsigned char param){
  dSPINA_Xfer(dSPIN_GET_PARAM | param);
  return dSPINA_ParamHandler(param, 0);
}

// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
unsigned long dSPINA_ParamHandler(unsigned char param, unsigned long value){
  unsigned long ret_val = 0;   // This is a temp for the value to return.
  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the dSPIN_Param()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  dSPIN_Xfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case dSPIN_ABS_POS:
      ret_val = dSPINA_Param(value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      ret_val = dSPINA_Param(value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case dSPIN_MARK:
      ret_val = dSPINA_Param(value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does
    //  NOT provide direction information.
    case dSPIN_SPEED:
      ret_val = dSPINA_Param(0, 20);
      break;
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case dSPIN_ACC:
      ret_val = dSPINA_Param(value, 12);
      break;
    case dSPIN_DEC:
      ret_val = dSPINA_Param(value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case dSPIN_MAX_SPEED:
      ret_val = dSPINA_Param(value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED:
      ret_val = dSPINA_Param(value, 12);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case dSPIN_FS_SPD:
      ret_val = dSPINA_Param(value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case dSPIN_KVAL_HOLD:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_RUN:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_ACC:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_DEC:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case dSPIN_INT_SPD:
      ret_val = dSPINA_Param(value, 14);
      break;
    case dSPIN_ST_SLP:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    case dSPIN_FN_SLP_ACC:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    case dSPIN_FN_SLP_DEC:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case dSPIN_K_THERM:
      ret_val = dSPINA_Xfer((unsigned char)value & 0x0F);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case dSPIN_ADC_OUT:
      ret_val = dSPINA_Xfer(0);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case dSPIN_OCD_TH:
      ret_val = dSPINA_Xfer((unsigned char)value & 0x0F);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case dSPIN_STALL_TH:
      ret_val = dSPINA_Xfer((unsigned char)value & 0x7F);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case dSPIN_STEP_MODE:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case dSPIN_ALARM_EN:
      ret_val = dSPINA_Xfer((unsigned char)value);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG:
      ret_val = dSPINA_Param(value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case dSPIN_STATUS:  // STATUS is a read-only register
      ret_val = dSPINA_Param(0, 16);
      break;
    default:
      ret_val = dSPINA_Xfer((unsigned char)(value));
      break;
  }
  return ret_val;
}

// Enable or disable the low-speed optimization option. If enabling,
//  the other 12 bits of the register will be automatically zero.
//  When disabling, the value will have to be explicitly written by
//  the user with a SetParam() call. See the datasheet for further
//  information about low-speed optimization.
void SetLSPDOptA(unsigned char enable){
  dSPINA_Xfer(dSPIN_SET_PARAM | dSPIN_MIN_SPEED);
  if (enable) dSPINA_Param(0x1000, 13);
  else dSPINA_Param(0, 13);
}

// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The SpdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void dSPINA_Run(unsigned char dir, unsigned long spd){
  dSPINA_Xfer(dSPIN_RUN | dir);
  if (spd > 0xFFFFF) spd = 0xFFFFF;
  dSPINA_Xfer((unsigned char)(spd >> 16));
  dSPINA_Xfer((unsigned char)(spd >> 8));
  dSPINA_Xfer((unsigned char)(spd));
}

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.
void dSPINA_Step_Clock(unsigned char dir){
  dSPINA_Xfer(dSPIN_STEP_CLOCK | dir);
}

// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void dSPINA_Move(unsigned char dir, unsigned long n_step){
  dSPINA_Xfer(dSPIN_MOVE | dir);
  if (n_step > 0x3FFFFF) n_step = 0x3FFFFF;
  dSPINA_Xfer((unsigned char)(n_step >> 16));
  dSPINA_Xfer((unsigned char)(n_step >> 8));
  dSPINA_Xfer((unsigned char)(n_step));
  dSPINA_Xfer(0);
}

// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void dSPINA_GoTo(unsigned long pos){
  dSPINA_Xfer(dSPIN_GOTO);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPINA_Xfer((unsigned char)(pos >> 16));
  dSPINA_Xfer((unsigned char)(pos >> 8));
  dSPINA_Xfer((unsigned char)(pos));
}

// Same as GOTO, but with user constrained rotational direction.
void dSPINA_GoTo_DIR(unsigned char dir, unsigned long pos){
  dSPINA_Xfer(dSPIN_GOTO_DIR);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPINA_Xfer((unsigned char)(pos >> 16));
  dSPINA_Xfer((unsigned char)(pos >> 8));
  dSPINA_Xfer((unsigned char)(pos));
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void dSPINA_GoUntil(unsigned char act, unsigned char dir, unsigned long spd){
  dSPINA_Xfer(dSPIN_GO_UNTIL | act | dir);
  if (spd > 0x3FFFFF) spd = 0x3FFFFF;
  dSPINA_Xfer((unsigned char)(spd >> 16));
  dSPINA_Xfer((unsigned char)(spd >> 8));
  dSPINA_Xfer((unsigned char)(spd));
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void dSPINA_ReleaseSW(unsigned char act, unsigned char dir){
  dSPINA_Xfer(dSPIN_RELEASE_SW | act | dir);
}

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPINA_GoHome(){
  dSPINA_Xfer(dSPIN_GO_HOME);
}

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPINA_GoMark(){
  dSPINA_Xfer(dSPIN_GO_MARK);
}

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void dSPINA_ResetPos(){
  dSPINA_Xfer(dSPIN_RESET_POS);
}

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void dSPINA_ResetDev(){
  dSPINA_Xfer(dSPIN_RESET_DEVICE);
}

// Bring the motor to a halt using the deceleration curve.
void dSPINA_SoftStop(){
  dSPINA_Xfer(dSPIN_SOFT_STOP);
}

// Stop the motor with infinite deceleration.
void dSPINA_HardStop(){
  dSPINA_Xfer(dSPIN_HARD_STOP);
}

// Decelerate the motor and put the bridges in Hi-Z state.
void dSPINA_SoftHiZ(){
  dSPINA_Xfer(dSPIN_SOFT_HIZ);
}

// Put the bridges in Hi-Z state immediately with no deceleration.
void dSPINA_HardHiZ(){
  dSPINA_Xfer(dSPIN_HARD_HIZ);
}

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
int dSPINA_GetStatus(){
  int temp = 0;
  dSPINA_Xfer(dSPIN_GET_STATUS);
  dSPINA_Xfer(0);
  temp = ((int)dSPINA_Xfer(0))<<8;
  temp |= (int)dSPINA_Xfer(0);
  return temp;
}

// MOTOR B

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void dSPINB_SetParam(unsigned char param, unsigned long value){
  dSPINB_Xfer(dSPIN_SET_PARAM | param);
  dSPINB_ParamHandler(param, value);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
unsigned long dSPINB_GetParam(unsigned char param){
  dSPINB_Xfer(dSPIN_GET_PARAM | param);
  return dSPINB_ParamHandler(param, 0);
}

// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
unsigned long dSPINB_ParamHandler(unsigned char param, unsigned long value){
  unsigned long ret_val = 0;   // This is a temp for the value to return.
  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the dSPIN_Param()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  dSPIN_Xfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case dSPIN_ABS_POS:
      ret_val = dSPINB_Param(value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      ret_val = dSPINB_Param(value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case dSPIN_MARK:
      ret_val = dSPINB_Param(value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does
    //  NOT provide direction information.
    case dSPIN_SPEED:
      ret_val = dSPINB_Param(0, 20);
      break;
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case dSPIN_ACC:
      ret_val = dSPINB_Param(value, 12);
      break;
    case dSPIN_DEC:
      ret_val = dSPINB_Param(value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case dSPIN_MAX_SPEED:
      ret_val = dSPINB_Param(value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED:
      ret_val = dSPINB_Param(value, 12);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case dSPIN_FS_SPD:
      ret_val = dSPINB_Param(value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case dSPIN_KVAL_HOLD:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_RUN:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_ACC:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    case dSPIN_KVAL_DEC:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case dSPIN_INT_SPD:
      ret_val = dSPINB_Param(value, 14);
      break;
    case dSPIN_ST_SLP:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    case dSPIN_FN_SLP_ACC:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    case dSPIN_FN_SLP_DEC:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case dSPIN_K_THERM:
      ret_val = dSPINB_Xfer((unsigned char)value & 0x0F);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case dSPIN_ADC_OUT:
      ret_val = dSPINB_Xfer(0);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case dSPIN_OCD_TH:
      ret_val = dSPINB_Xfer((unsigned char)value & 0x0F);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case dSPIN_STALL_TH:
      ret_val = dSPINB_Xfer((unsigned char)value & 0x7F);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case dSPIN_STEP_MODE:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case dSPIN_ALARM_EN:
      ret_val = dSPINB_Xfer((unsigned char)value);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG:
      ret_val = dSPINB_Param(value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case dSPIN_STATUS:  // STATUS is a read-only register
      ret_val = dSPINB_Param(0, 16);
      break;
    default:
      ret_val = dSPINB_Xfer((unsigned char)(value));
      break;
  }
  return ret_val;
}

// Enable or disable the low-speed optimization option. If enabling,
//  the other 12 bits of the register will be automatically zero.
//  When disabling, the value will have to be explicitly written by
//  the user with a SetParam() call. See the datasheet for further
//  information about low-speed optimization.
void SetLSPDOptB(unsigned char enable){
  dSPINB_Xfer(dSPIN_SET_PARAM | dSPIN_MIN_SPEED);
  if (enable) dSPINB_Param(0x1000, 13);
  else dSPINB_Param(0, 13);
}

// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The SpdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void dSPINB_Run(unsigned char dir, unsigned long spd){
  dSPINB_Xfer(dSPIN_RUN | dir);
  if (spd > 0xFFFFF) spd = 0xFFFFF;
  dSPINB_Xfer((unsigned char)(spd >> 16));
  dSPINB_Xfer((unsigned char)(spd >> 8));
  dSPINB_Xfer((unsigned char)(spd));
}

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.
void dSPINB_Step_Clock(unsigned char dir){
  dSPINB_Xfer(dSPIN_STEP_CLOCK | dir);
}

// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void dSPINB_Move(unsigned char dir, unsigned long n_step){
  dSPINB_Xfer(dSPIN_MOVE | dir);
  if (n_step > 0x3FFFFF) n_step = 0x3FFFFF;
  dSPINB_Xfer((unsigned char)(n_step >> 16));
  dSPINB_Xfer((unsigned char)(n_step >> 8));
  dSPINB_Xfer((unsigned char)(n_step));
  dSPINB_Xfer(0);
}

// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void dSPINB_GoTo(unsigned long pos){
  dSPINB_Xfer(dSPIN_GOTO);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPINB_Xfer((unsigned char)(pos >> 16));
  dSPINB_Xfer((unsigned char)(pos >> 8));
  dSPINB_Xfer((unsigned char)(pos));
}

// Same as GOTO, but with user constrained rotational direction.
void dSPINB_GoTo_DIR(unsigned char dir, unsigned long pos){
  dSPINB_Xfer(dSPIN_GOTO_DIR);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPINB_Xfer((unsigned char)(pos >> 16));
  dSPINB_Xfer((unsigned char)(pos >> 8));
  dSPINB_Xfer((unsigned char)(pos));
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void dSPINB_GoUntil(unsigned char act, unsigned char dir, unsigned long spd){
  dSPINB_Xfer(dSPIN_GO_UNTIL | act | dir);
  if (spd > 0x3FFFFF) spd = 0x3FFFFF;
  dSPINB_Xfer((unsigned char)(spd >> 16));
  dSPINB_Xfer((unsigned char)(spd >> 8));
  dSPINB_Xfer((unsigned char)(spd));
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void dSPINB_ReleaseSW(unsigned char act, unsigned char dir){
  dSPINB_Xfer(dSPIN_RELEASE_SW | act | dir);
}

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPINB_GoHome(){
  dSPINB_Xfer(dSPIN_GO_HOME);
}

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPINB_GoMark(){
  dSPINB_Xfer(dSPIN_GO_MARK);
}

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void dSPINB_ResetPos(){
  dSPINB_Xfer(dSPIN_RESET_POS);
}

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void dSPINB_ResetDev(){
  dSPINB_Xfer(dSPIN_RESET_DEVICE);
}

// Bring the motor to a halt using the deceleration curve.
void dSPINB_SoftStop(){
  dSPINB_Xfer(dSPIN_SOFT_STOP);
}

// Stop the motor with infinite deceleration.
void dSPINB_HardStop(){
  dSPINB_Xfer(dSPIN_HARD_STOP);
}

// Decelerate the motor and put the bridges in Hi-Z state.
void dSPINB_SoftHiZ(){
  dSPINB_Xfer(dSPIN_SOFT_HIZ);
}

// Put the bridges in Hi-Z state immediately with no deceleration.
void dSPINB_HardHiZ(){
  dSPINB_Xfer(dSPIN_HARD_HIZ);
}

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
int dSPINB_GetStatus(){
  int temp = 0;
  dSPINB_Xfer(dSPIN_GET_STATUS);
  dSPINB_Xfer(0);
  temp = ((int) dSPINB_Xfer(0))<<8;
  temp |= (int) dSPINB_Xfer(0);
  return temp;
}

//------------------------------------------------------------------------------------------
//   END COMMAND CODE ADAPTED FROM dSPIN_command.ino
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//   BEGIN SUPPORT CODE ADAPTED FROM dSPIN_support.ino
//   Contains functions used to implement the high-level commands,
//   as well as utility functions for converting real-world units (eg, steps/s) to
//   values usable by the dsPIN controller. Also contains the specialized configuration
//   function for the dsPIN chip and the onboard peripherals needed to use it.
//------------------------------------------------------------------------------------------

// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is
//  250ns (datasheet value)- 0x08A on boot.
// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long AccCalc(float stepsPerSecPerSec){
  float temp = stepsPerSecPerSec * 0.0687195;
  if( (unsigned long) round(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) round(temp);
}

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long DecCalc(float stepsPerSecPerSec){
  float temp = stepsPerSecPerSec * 0.0687195;
  if( (unsigned long) round(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) round(temp);
}

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long MaxSpdCalc(float stepsPerSec){
  float temp = stepsPerSec * .065536;
  if( (unsigned long) round(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) round(temp);
}

// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long MinSpdCalc(float stepsPerSec){
  float temp = stepsPerSec * 4.194304;
  if( (unsigned long) round(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) round(temp);
}

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long FSCalc(float stepsPerSec){
  float temp = (stepsPerSec * .065536)-.5;
  if( (unsigned long) round(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) round(temp);
}

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long IntSpdCalc(float stepsPerSec){
  float temp = stepsPerSec * 4.194304;
  if( (unsigned long) round(temp) > 0x00003FFF) return 0x00003FFF;
  else return (unsigned long) round(temp);
}

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long SpdCalc(float stepsPerSec){
  float temp = stepsPerSec * 67.106;
  if( (unsigned long) round(temp) > 0x000FFFFF) return 0x000FFFFF;
  else return (unsigned long) round(temp);
}

// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
unsigned long dSPINA_Param(unsigned long value, unsigned char bit_len){
	unsigned long ret_val = 0;    // We'll return this to generalize this function
								  //  for both read and write of registers.
	unsigned char byte_len = bit_len/8;      // How many BYTES do we have?
	if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
	// Let's make sure our value has no spurious bits set, and if the value was too
	//  high, max it out.
	unsigned long mask = 0xffffffff >> (32 - bit_len);
	if (value > mask) value = mask;
	// The following three if statements handle the various possible byte length
	//  transfers- it'll be no less than 1 but no more than 3 bytes of data.
	// dSPIN_Xfer() sends a byte out through SPI and returns a byte received
	//  over SPI- when calling it, we typecast a shifted version of the masked
	//  value, then we shift the received value back by the same amount and
	//  store it until return time.
	if (byte_len == 3) ret_val |= (unsigned long)dSPINA_Xfer((unsigned char)(value>>16)) << 16;
	if (byte_len >= 2) ret_val |= (unsigned long)dSPINA_Xfer((unsigned char)(value>>8)) << 8;
	if (byte_len >= 1) ret_val |= (unsigned long)dSPINA_Xfer((unsigned char)value);

	// Return the received values. Mask off any unnecessary bits, just for
	//  the sake of thoroughness- we don't EXPECT to see anything outside
	//  the bit length range but better to be safe than sorry.
	return (ret_val & mask);
}

unsigned long dSPINB_Param(unsigned long value, unsigned char bit_len){
	unsigned long ret_val = 0;    // We'll return this to generalize this function
								  //  for both read and write of registers.
	unsigned char byte_len = bit_len/8;      // How many BYTES do we have?
	if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
	// Let's make sure our value has no spurious bits set, and if the value was too
	//  high, max it out.
	unsigned long mask = 0xffffffff >> (32 - bit_len);
	if (value > mask) value = mask;
	// The following three if statements handle the various possible byte length
	//  transfers- it'll be no less than 1 but no more than 3 bytes of data.
	// dSPIN_Xfer() sends a byte out through SPI and returns a byte received
	//  over SPI- when calling it, we typecast a shifted version of the masked
	//  value, then we shift the received value back by the same amount and
	//  store it until return time.
	if (byte_len == 3) ret_val |= (unsigned long)dSPINB_Xfer((unsigned char)(value>>16)) << 16;
	if (byte_len >= 2) ret_val |= (unsigned long)dSPINB_Xfer((unsigned char)(value>>8)) << 8;
	if (byte_len >= 1) ret_val |= (unsigned long)dSPINB_Xfer((unsigned char)value);

	// Return the received values. Mask off any unnecessary bits, just for
	//  the sake of thoroughness- we don't EXPECT to see anything outside
	//  the bit length range but better to be safe than sorry.
	return (ret_val & mask);
}

// This simple function shifts a byte out over SPI and receives a byte over
//  SPI. Unusually for SPI devices, the dSPIN requires a toggling of the
//  CS (slaveSelect) pin after each byte sent. That makes this function
//  a bit more reasonable, because we can include more functionality in it.
unsigned char dSPINA_Xfer(unsigned char data){
	unsigned long data_out;
	unsigned char returnvalue;

	//motor A Select
	//20ns per cycle at 50MHz system clock
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	SysCtlDelay(100); // delay of 5.833 should create the minimum 350 ns delay required after chip select, SysCtlDelay takes 3 cycles
	SSIDataPut(SSI1_BASE, data);
	SSIDataGetNonBlocking(SSI1_BASE, &data_out);
	//motor A Deselect
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
	SysCtlDelay(250); // delay 13.33 should create the minimum 800 ns delay after each byte sent, SysCtlDelay takes 3 cycles

	returnvalue = (unsigned char) (data_out & 0x000F);
	return returnvalue;
}

unsigned char dSPINB_Xfer(unsigned char data){
	unsigned long data_out;
	unsigned char returnvalue;

	//motor B Select
	//20ns per cycle at 50MHz system clock
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
	SysCtlDelay(100); // delay of 5.833 should create the minimum 350 ns delay required after chip select, SysCtlDelay takes 3 cycles
	SSIDataPut(SSI2_BASE, data);
	SSIDataGetNonBlocking(SSI2_BASE, &data_out);
	//motor B Deselect
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
	SysCtlDelay(250); // delay 13.33 should create the minimum 800 ns delay after each byte sent, SysCtlDelay takes 3 cycles

	returnvalue = (unsigned char) (data_out & 0x00FF);
	return returnvalue;
}

//------------------------------------------------------------------------------------------
//   END SUPPORT CODE ADAPTED FROM dSPIN_support.ino
//------------------------------------------------------------------------------------------

// Initializes the SPI Interfaces in the Stellaris ARM
// and sets up register values for the motor controller.
// Must be called before any other motor code is used.
void dSPINinit(){
// Mateus

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//SSI2, Motor Control B, right hand side, Pins 1,4,57,58
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//SSI1, Motor Control A, left hand side, Pins 28-31

	//Unlock and commit NMI pins PD7 and PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0x4C4F434B;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x1;
	//HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;
	//HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;


	//Pin 1 -> M_B_MISO -> PB6
	//Pin 4 -> M_B_MOSI -> PB7
	//Pin 57 -> M_B_CS (SSI2 Fss) -> PB5
	//Pin 58 -> M_B_SCLK (SSI2) -> PB4
	//Pin 48 -> M_B_RESET -> PB3

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB6_SSI2RX);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);
	GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);
	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
			SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI2_BASE);
	//Chip Select GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	//Reset Toggle GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);


	//Pin 28 -> M_A_MISO -> PF0
	//Pin 29 -> M_A_MOSI -> PF1
	//Pin 30 -> M_A_SCLK (SSI1) -> PF2
	//Pin 31 -> M_A_CS (SSI1 Fss) -> PF3
	//Pin 5 -> M_A_RESET -> PF4

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	GPIOPinConfigure(GPIO_PF0_SSI1RX);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
			SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI1_BASE);
	//Chip Select GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	//Reset Toggle GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);


	dSPINreset();

	//Set Current Limits
	/*dSPINA_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_3750mA);
	dSPINB_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_3750mA);
	dSPINA_SetParam(dSPIN_STALL_TH, 0x6A);
	dSPINB_SetParam(dSPIN_STALL_TH, 0x6A);*/

	dSPINA_SetParam(dSPIN_STEP_MODE, dSPIN_STEP_SEL_1);
	dSPINB_SetParam(dSPIN_STEP_MODE, dSPIN_STEP_SEL_1);
	dSPINA_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(200));
	dSPINB_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(200));
	dSPINA_SetParam(dSPIN_FS_SPD, FSCalc(150));
	dSPINB_SetParam(dSPIN_FS_SPD, FSCalc(150));

}

// Toggles reset pins to reset motor controller chip.
// Called automatically in dSPINinit(), so shouldn't be
// needed to call manually unless something goes wrong.
void dSPINreset(){
//Mateus
	//reset Motor B
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
	SysCtlDelay(1000);
	//reset Motor A
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
	SysCtlDelay(1000);
}

// Checks if both motors are ready to move.
// Returns 1 if motors are ready.
// Returns 0 if motors are still not ready after dSPIN_TIMEOUT_SEC seconds.
int dSPIN_Ready(){
//Mateus
	// Make sure motors are ready to move
	int Ready = 0, temp = 0,
			StatusA, StatusB;

	do{
		SysCtlDelay(500 * dSPIN_STEL_MS); //poll only once every 500 ms
		StatusA = dSPINA_GetStatus();
		StatusB = dSPINB_GetStatus();
		if(StatusA & dSPIN_STATUS_MOT_STATUS == dSPIN_STATUS_MOT_STATUS_STOPPED
				&& StatusB & dSPIN_STATUS_MOT_STATUS == dSPIN_STATUS_MOT_STATUS_STOPPED
				//&& etc.
				//TODO: CHECK MORE STATUS, BUSY SIGNAL?

		)Ready = 1;
		temp++;
	}while(!Ready && temp < dSPIN_TIMEOUT_SEC * 2); // temp counts in 500 ms chunks

	return Ready;
}

int MoveTiles{
  /*Proprietary Magic*/
}


