package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Intake extends SmartSubsystem {
  private final VictorSPX motor;
  private final DoubleSolenoid pneumaticControl, solenoid2;

  public Intake() {
    motor = new VictorSPX(Constants.CAN.INTAKE);
    pneumaticControl = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.INTAKE_L_OUT, Constants.Solenoid.INTAKE_L_IN);
    solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.INTAKE_R_OUT, Constants.Solenoid.INTAKE_R_IN);

    motor.setNeutralMode(NeutralMode.Coast);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(.25);
    // TODO supply current limiting
  }

  public void setIntakeSpeed (double intakeSpeed) {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void setExtend (boolean extend) {
    if (extend) {
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

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareTalon(this, motor));
    Test.checkSolenoid(this, pneumaticControl);
    Test.checkSolenoid(this, solenoid2);
  }
}
