package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team4646.Test;

public class Intake extends SmartSubsystem {
  private class DataCache {
    public boolean extended = true;
  }
  private class OutputCache {
    public double setpoint = 0.0;
    public boolean extend = false;
  }
  private final TalonFX motor;
  private final DoubleSolenoid solenoid;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

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

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, Constants.CAN_TIMEOUT);
    setExtend(!cache.extended);  // solenoid default is OFF, not IN
  }

  @Override
  public void updateHardware() {
    updateSolenoids();
    updateMotors();
  }

  public void setOpenLoop (double intakeSpeed) { outputs.setpoint = intakeSpeed; }
  public void setExtend (boolean extend) { outputs.extend = extend; }
  public boolean isExtended() { return cache.extended; }

  private void updateMotors() {
    motor.set(ControlMode.PercentOutput, outputs.setpoint);
  }

  private void updateSolenoids() {
    if (outputs.extend != cache.extended) {
      cache.extended = outputs.extend;
      solenoid.set(cache.extended ? Value.kForward : Value.kReverse);
    }
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
    Test.checkSolenoid(this, solenoid);
    Test.checkStatusFrames(this, motor);
    Test.add(this, "Extended", !cache.extended);
  }
}
