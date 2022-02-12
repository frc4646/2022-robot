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
  private final DoubleSolenoid solenoidL, solenoidR;

  private boolean extended = false;

  public Intake() {
    motor = new VictorSPX(Constants.CAN.INTAKE);
    solenoidL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.INTAKE_L_OUT, Constants.Solenoid.INTAKE_L_IN);
    solenoidR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.INTAKE_R_OUT, Constants.Solenoid.INTAKE_R_IN);

    motor.setNeutralMode(NeutralMode.Coast);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Constants.Intake.OPEN_LOOP_RAMP);
    // TODO supply current limiting
  }

  public void setIntakeSpeed (double intakeSpeed) {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void setExtend (boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    solenoidL.set(direction);
    solenoidR.set(direction);
    extended = extend;
  }

  public boolean isExtended() {
    return extended;
  }

  public boolean isStalled() {
    return false;  // TODO need talon/sparkmax to detect
  }

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareTalon(this, motor));
    Test.checkSolenoid(this, solenoidL);
    Test.checkSolenoid(this, solenoidR);
  }
}
