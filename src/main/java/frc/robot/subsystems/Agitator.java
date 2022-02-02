package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Agitator extends SmartSubsystem {
  private final VictorSPX motor;

  public Agitator() {
    motor = new VictorSPX(Constants.Ports.AGITATOR);
    motor.configOpenloopRamp(.25);
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }
}
