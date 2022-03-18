package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team4646.Test;

public class Intake extends SmartSubsystem {
  private final TalonFX motor;
  private final DoubleSolenoid solenoid;

  private boolean extended = false;

  public Intake() {
    motor = TalonFXFactory.createDefaultTalon(Constants.CAN.INTAKE);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID.INTAKE_OUT, Constants.SOLENOID.INTAKE_IN);

    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Constants.INTAKE.OPEN_LOOP_RAMP);
    // StatorCurrentLimitConfiguration limit = new StatorCurrentLimitConfiguration(true, 30.0, 50.0, 0.02);
    // TalonUtil.checkError(motor.configStatorCurrentLimit(limit), "Intake: Could not set stator current limit");

    setExtend(false);  // solenoid default is OFF, not IN
  }

  public void setOpenLoop (double intakeSpeed) {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void setExtend (boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    solenoid.set(direction);
    extended = extend;
  }

  public boolean isExtended() {
    return extended;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
    Test.checkSolenoid(this, solenoid);
    Test.checkStatusFrames(this, motor);
  }
}
