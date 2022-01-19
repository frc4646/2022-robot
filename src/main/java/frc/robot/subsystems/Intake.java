package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final VictorSPX motor;

  public Intake() {
    motor = new VictorSPX(Constants.Ports.INTAKE);
    // TODO supply current limiting
  }

  public void setIntakeSpeed () {
    motor.set(ControlMode.PercentOutput, .5);
  }

  public boolean isStalled() {
    return false;  // TODO need talon/sparkmax to detect
  }
}
