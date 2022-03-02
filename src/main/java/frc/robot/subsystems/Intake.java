package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.TalonFXFactory;

public class Intake extends SmartSubsystem {
  private final TalonFX motor;
  private final DoubleSolenoid solenoid;

  private boolean extended = false;

  public Intake() {
    motor = TalonFXFactory.createDefaultTalon(Constants.CAN.INTAKE);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.INTAKE_OUT, Constants.Solenoid.INTAKE_IN);

    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Constants.Intake.OPEN_LOOP_RAMP);
    // TODO supply current limiting

    setExtend(false);  // solenoid default is OFF, not IN
  }

  public void setIntakeSpeed (double intakeSpeed) {
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
  }
}
