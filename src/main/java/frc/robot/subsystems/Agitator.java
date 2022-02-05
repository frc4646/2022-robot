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
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(.25);
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareTalon(this, motor));
  }
}
