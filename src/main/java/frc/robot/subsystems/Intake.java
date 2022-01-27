package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class Intake extends SmartSubsystem {
  private final VictorSPX motor;
  // TODO add pneumatics

  public Intake() {
    motor = new VictorSPX(Constants.Ports.INTAKE);
    // TODO supply current limiting
    // TODO create double solnoid
  }

  public void setIntakeSpeed (double intakeSpeed) {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void extendIntake (boolean extend) {
    // TODO if true extend pnuematics and intake
    // TODO if false retract Pnuematics and intake
  }

  public boolean isStalled() {
    return false;  // TODO need talon/sparkmax to detect
  }
}
