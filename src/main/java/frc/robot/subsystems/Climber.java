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
    public double positionL, velocityL, currentL, positionR, velocityR, currentR;
  }

  private final TalonFX masterL, masterR;
  private final DoubleSolenoid armL, armR;
  private final DataCache cache = new DataCache();
  private boolean isBrakeMode = false, armsExtended = false, mHasBeenZeroedL = false, mHasBeenZeroedR = false, inClimbMode = false;

  public Climber() {
    masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_L);
    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_R);
    armL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_L_OUT, Constants.Solenoid.ARM_L_IN);
    armR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_R_OUT, Constants.Solenoid.ARM_R_IN);

    configureMotor(masterL, true, false);
    configureMotor(masterR, true, true);

    isBrakeMode = false;
    setBrakeMode(true);
    setArms(false);  // solenoid default is OFF, not IN
    setSoftLimitsEnabled(true);
    forceZero(true);
    forceZero(false);
  }

  protected void configureMotor(TalonFX motor, boolean isMaster, boolean isInverted) {
    TalonUtil.checkError(motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set reverse limit switch: ");
    if (isMaster) {
      TalonUtil.checkError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), getName() + ": Could not detect encoder: ");
      TalonUtil.checkError(motor.configForwardSoftLimitThreshold(Constants.Climber.LIMIT_F, Constants.CAN_TIMEOUT), getName() + ": Could not set forward soft limit: ");
      TalonUtil.checkError(motor.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT), getName() + ": Could not enable forward soft limit: ");
      TalonUtil.checkError(motor.configReverseSoftLimitThreshold(0.0, Constants.CAN_TIMEOUT), getName() + ": Could not set reverse soft limit: ");
      TalonUtil.checkError(motor.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT), getName() + ": Could not enable reverse soft limit: ");
    }

    StatorCurrentLimitConfiguration limitStator = new StatorCurrentLimitConfiguration(true, 60, 60, 0.2);
    TalonUtil.checkError(motor.configStatorCurrentLimit(limitStator, Constants.CAN_TIMEOUT), getName() + ": Could not set stator current limits");
    TalonUtil.checkError(motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT), getName() + ": Could not set voltage comp saturation");
    motor.enableVoltageCompensation(true);

    motor.setInverted(isInverted);
    if (isMaster) {
      // master.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, Constants.CAN_TIMEOUT);
    }
    motor.overrideLimitSwitchesEnable(true);
  }

  @Override
  public void cacheSensors() {
    cache.limitL = masterL.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cache.positionL = masterL.getSelectedSensorPosition(0);
    cache.velocityL = masterL.getSelectedSensorVelocity(0);
    cache.currentL = masterL.getStatorCurrent();
    cache.limitR = masterR.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cache.positionR = masterR.getSelectedSensorPosition(0);
    cache.velocityR = masterR.getSelectedSensorVelocity(0);
    cache.currentR = masterR.getStatorCurrent();
    resetIfAtHome();  // TODO test this
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climber: Limit L", cache.limitL);
    SmartDashboard.putBoolean("Climber: Limit R", cache.limitR);
    if (Constants.Climber.TUNING) {
      SmartDashboard.putNumber("Climber: PositionL", cache.positionL);
      SmartDashboard.putNumber("Climber: VelocityL", cache.velocityL);
      SmartDashboard.putNumber("Climber: CurrentL", cache.currentL);
      SmartDashboard.putNumber("Climber: PositionR", cache.positionR);
      SmartDashboard.putNumber("Climber: VelocityR", cache.velocityR);
      SmartDashboard.putNumber("Climber: CurrentR", cache.currentR);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  public void resetIfAtHome() {
    if (atHomingLocation(true)) {
      zeroSensors(true);
    }
    if (atHomingLocation(false)) {
      zeroSensors(false);
    }
  }

  public void zeroSensors(boolean left) {
    forceZero(left);
    if (left) {
      mHasBeenZeroedL = true;
    } else {      
      mHasBeenZeroedR = true;
    }
  }

  public void forceZero(boolean left) {
    TalonFX motor = (left) ? masterL : masterR;
    motor.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    cache.positionL = motor.getSelectedSensorPosition(0);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;  // Already in this mode
    }
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    masterL.setNeutralMode(mode);
    masterR.setNeutralMode(mode);
    isBrakeMode = enable;
  }

  public void setSoftLimitsEnabled(boolean enable) {
    masterL.overrideSoftLimitsEnable(enable);
    masterR.overrideSoftLimitsEnable(enable);
  }

  public void setOpenLoop(double percent) {
    masterL.set(TalonFXControlMode.PercentOutput, percent);
    masterR.set(TalonFXControlMode.PercentOutput, percent);
  }

  public void setArms(boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    armL.set(direction);
    armR.set(direction);
    armsExtended = extend;
  }

  public void setClimbMode(boolean enable) {
    inClimbMode = enable;
  }

  public boolean isArmsExtended() { return armsExtended; }
  public boolean isInClimbMode() { return inClimbMode; }
  public boolean hasBeenZeroed() { return mHasBeenZeroedL && mHasBeenZeroedR; }
  public boolean atHomingLocation(boolean left) { return (left) ? cache.limitL : cache.limitR; }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkSolenoid(this, armL);
    Test.checkSolenoid(this, armR);
    Test.add(this, "Limit L", cache.limitL);
    Test.add(this, "Limit R", cache.limitR);
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, masterR);
  }
}
