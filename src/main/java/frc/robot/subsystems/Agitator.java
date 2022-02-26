package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.util.Test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Agitator extends SmartSubsystem {
  private final VictorSPX motorL, motorR;

  public Agitator() {
    motorL = new VictorSPX(Constants.CAN.AGITATOR_L);
    motorL.setInverted(true);
    motorL.setNeutralMode(NeutralMode.Brake);
    motorL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motorL.enableVoltageCompensation(true);
    motorL.configOpenloopRamp(Constants.Agitator.OPEN_LOOP_RAMP);

    motorR = new VictorSPX(Constants.CAN.AGITATOR_R);
    motorR.setInverted(false);
    motorR.setNeutralMode(NeutralMode.Brake);
    motorR.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motorR.enableVoltageCompensation(true);
    motorR.configOpenloopRamp(Constants.Agitator.OPEN_LOOP_RAMP);

  }

  public void setOpenLoop(double percent) {
    motorL.set(ControlMode.PercentOutput, percent);
    motorR.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motorL);
    Test.checkFirmware(this, motorR);
  }
}
