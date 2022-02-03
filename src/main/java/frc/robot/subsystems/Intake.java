package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends SmartSubsystem {
  private final VictorSPX motor;
  private final DoubleSolenoid pneumaticControl;
  private final DoubleSolenoid solenoid2;

  public Intake() {
    motor = new VictorSPX(Constants.Ports.INTAKE);
    pneumaticControl = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    motor.setNeutralMode(NeutralMode.Coast);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(.25);
    // TODO supply current limiting
  }

  public void setIntakeSpeed (double intakeSpeed) {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void extendIntake (boolean extend) {
    if (extend == true) {
      solenoid2.set(Value.kForward);
      pneumaticControl.set(Value.kForward);
    }
    else {
      solenoid2.set(Value.kReverse);
      pneumaticControl.set(Value.kReverse);
    }
  }

  public boolean isStalled() {
    return false;  // TODO need talon/sparkmax to detect
  }
}
