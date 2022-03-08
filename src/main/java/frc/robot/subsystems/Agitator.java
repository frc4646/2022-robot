package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.util.Test;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

public class Agitator extends SmartSubsystem {
  private final VictorSPX masterL, slaveR;

  public Agitator() {
    masterL = new VictorSPX(Constants.CAN.AGITATOR_L);
    masterL.setInverted(true);
    masterL.setNeutralMode(NeutralMode.Brake);
    masterL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    masterL.enableVoltageCompensation(true);
    masterL.configOpenloopRamp(Constants.AGITATOR.OPEN_LOOP_RAMP);
    masterL.setStatusFramePeriod(StatusFrame.Status_1_General, 10, Constants.CAN_TIMEOUT);
    masterL.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, Constants.CAN_TIMEOUT);
    masterL.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000, Constants.CAN_TIMEOUT);
    masterL.setControlFramePeriod(ControlFrame.Control_3_General, 10);

    slaveR = new VictorSPX(Constants.CAN.AGITATOR_R);
    slaveR.follow(masterL);
    slaveR.setInverted(false);
    slaveR.setNeutralMode(NeutralMode.Brake);
    slaveR.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    slaveR.enableVoltageCompensation(true);
    slaveR.configOpenloopRamp(Constants.AGITATOR.OPEN_LOOP_RAMP);    
    slaveR.setStatusFramePeriod(StatusFrame.Status_1_General, 1000, Constants.CAN_TIMEOUT);
    slaveR.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000, Constants.CAN_TIMEOUT);
    slaveR.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000, Constants.CAN_TIMEOUT);
    slaveR.setControlFramePeriod(ControlFrame.Control_3_General, 100);
  }

  public void setOpenLoop(double percent) {
    masterL.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, slaveR);
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, slaveR);
  }
}
