package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;

public class Climber extends SmartSubsystem {
  private static class DataCache {
    public boolean limitL, limitR;
    public double position, velocity, current;
  }

  private final TalonFX master, slave;
  // private final DoubleSolenoid armL, armR;//, ratchetL, ratchetR;
  private final DataCache cache = new DataCache();

  private boolean isBrakeMode = false, armsExtended = false, ratchetEngaged = false;

  public Climber() {
    master = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_L);
    slave = TalonFXFactory.createPermanentSlaveTalon(Constants.CAN.CLIMBER_R, Constants.CAN.CLIMBER_L);
    // armL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_L_OUT, Constants.Solenoid.ARM_L_IN);
    // armR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_R_OUT, Constants.Solenoid.ARM_R_IN);
    // ratchetL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.RATCHET_L_OUT, Constants.Solenoid.RATCHET_L_IN);
    // ratchetR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.RATCHET_R_OUT, Constants.Solenoid.RATCHET_R_IN);

    configureMotor(master, true, false);
    configureMotor(slave, false, true);

    isBrakeMode = false;
    setBrakeMode(true);
    // setArms(false);  // solenoid default is OFF, not IN
    setSoftLimitsEnabled(false);  // TODO enable
  }

  protected void configureMotor(TalonFX motor, boolean isMaster, boolean isInverted) {
    TalonUtil.checkError(slave.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set reverse limit switch: ");
    if (isMaster) {
      TalonUtil.checkError(master.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), getName() + ": Could not detect encoder: ");
      TalonUtil.checkError(master.configForwardSoftLimitThreshold(Constants.Climber.LIMIT_F, Constants.CAN_TIMEOUT), getName() + ": Could not set forward soft limit: ");
      TalonUtil.checkError(master.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT), getName() + ": Could not enable forward soft limit: ");
    }

    StatorCurrentLimitConfiguration limitStator = new StatorCurrentLimitConfiguration(true, 60, 60, 0.2);
    TalonUtil.checkError(motor.configStatorCurrentLimit(limitStator, Constants.CAN_TIMEOUT), getName() + ": Could not set stator current limits");
    TalonUtil.checkError(motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT), getName() + ": Could not set voltage comp saturation");
    motor.enableVoltageCompensation(true);

    motor.setInverted(isInverted);
    if (isMaster) {
      // master.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, Constants.CAN_TIMEOUT);
      master.setSelectedSensorPosition(0.0, 0, Constants.CAN_TIMEOUT);
    }
    motor.overrideLimitSwitchesEnable(true);
  }

  @Override
  public void cacheSensors() {
    cache.limitL = master.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cache.limitR = master.getSensorCollection().isRevLimitSwitchClosed() == 1;
    // cache.shift_out = mShiftSolenoidTimer.update(mShiftSolenoid.get(), 0.2);
    cache.position = master.getSelectedSensorPosition(0);
    cache.velocity = master.getSelectedSensorVelocity(0);
    cache.current = master.getStatorCurrent();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climber: Limit L", cache.limitL);
    SmartDashboard.putBoolean("Climber: Limit R", cache.limitR);
    if (Constants.Climber.TUNING) {
      SmartDashboard.putNumber("Climber: Position", cache.position);
      SmartDashboard.putNumber("Climber: Velocity", cache.velocity);
      SmartDashboard.putNumber("Climber: Current", cache.current);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;  // Already in this mode
    }
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    master.setNeutralMode(mode);
    slave.setNeutralMode(mode);
    isBrakeMode = enable;
  }

  public void setSoftLimitsEnabled(boolean enable) {
    master.overrideSoftLimitsEnable(enable);
  }

  public void setOpenLoop(double percent) {
    master.set(TalonFXControlMode.PercentOutput, percent);
  }

  public void setArms(boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    // armL.set(direction);
    // armR.set(direction);
    armsExtended = extend;
  }

  public void setRatchet(boolean engage) {
    Value direction = (engage) ? Value.kForward : Value.kReverse;
    // ratchetL.set(direction);
    // ratchetR.set(direction);
    ratchetEngaged = engage;
  }

  public boolean isArmsExtended() {
    return armsExtended;
  }

  public boolean isRatchetEngaged() {
    return ratchetEngaged;
  }

  public boolean isStalled() {
    return false;  // TODO zero if stalled at bottom rather than snap cable
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, master);
    Test.checkFirmware(this, slave);
    // Test.checkStatusFrames(master);
    // Test.checkStatusFrames(slave);
    // Test.checkSolenoid(this, armL);
    // Test.checkSolenoid(this, armR);
    // Test.checkSolenoid(this, ratchetL);
    // Test.checkSolenoid(this, ratchetR);
  }
}
