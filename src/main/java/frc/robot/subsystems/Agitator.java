package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.util.Test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Agitator extends SmartSubsystem {
  private final VictorSPX motor;

  public Agitator() {
    motor = new VictorSPX(Constants.CAN.AGITATOR);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Constants.Agitator.OPEN_LOOP_RAMP);
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
