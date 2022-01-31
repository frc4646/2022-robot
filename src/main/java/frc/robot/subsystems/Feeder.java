package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Feeder extends SmartSubsystem {
  private final VictorSPX motor;

  public Feeder() {
    motor = new VictorSPX(Constants.Ports.FEEDER);
    // TODO supply current limiting
    // TODO add sensor for if ball is in indexer
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }
}
