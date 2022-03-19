package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team4646.Test;

public class Climber extends SmartSubsystem {
  private class DataCache {
    public boolean limitL, limitR;
    public double positionL, positionR;
    public boolean inBrakeMode = false;
    public boolean extended = true;
  }
  private class OutputCache {
    public TalonFXControlMode mode = TalonFXControlMode.PercentOutput;
    public double setpointL = 0.0, setpointR = 0.0;
    public boolean extend = false;

    public void set(TalonFXControlMode mode, double setpointL, double setpointR) {
      outputs.mode = mode;
      outputs.setpointL = setpointL;
      outputs.setpointR = setpointR;
    }
  }

  private final TalonFX masterL, masterR;
  private final DoubleSolenoid armL, armR;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();
  private boolean hasBeenZeroedL = false, hasBeenZeroedR = false, inClimbMode = false;

  public Climber() {
    masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_L);
    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_R);
    armL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID.ARM_L_OUT, Constants.SOLENOID.ARM_L_IN);
    armR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID.ARM_R_OUT, Constants.SOLENOID.ARM_R_IN);

    configureMotor(masterL, false);
    configureMotor(masterR, true);

    updateBrakeMode(!cache.inBrakeMode);
    setArms(!cache.extended);  // solenoid default is OFF, not IN
    setSoftLimitsEnabled(true);
    forceZero(true);
    forceZero(false);
  }

  protected void configureMotor(TalonFX motor, boolean isInverted) {
    TalonUtil.checkError(motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Climber: Could not set reverse limit switch: ");
    TalonUtil.checkError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Climber: Could not detect encoder: ");
    TalonUtil.checkError(motor.configForwardSoftLimitThreshold(Constants.CLIMBER.LIMIT_F, Constants.CAN_TIMEOUT), "Climber: Could not set forward soft limit: ");
    TalonUtil.checkError(motor.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT), "Climber: Could not enable forward soft limit: ");
    TalonUtil.checkError(motor.configReverseSoftLimitThreshold(0.0, Constants.CAN_TIMEOUT), "Climber: Could not set reverse soft limit: ");
    TalonUtil.checkError(motor.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT), "Climber: Could not enable reverse soft limit: ");

    TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Climber: Could not detect encoder: ");
    TalonUtil.checkError(motor.config_kP(0, Constants.CLIMBER.P, Constants.CAN_TIMEOUT), "Climber: could not set P: ");
    TalonUtil.checkError(motor.config_kI(0, Constants.CLIMBER.I, Constants.CAN_TIMEOUT), "Climber: could not set I: ");
    TalonUtil.checkError(motor.config_kD(0, Constants.CLIMBER.D, Constants.CAN_TIMEOUT), "Climber: could not set D: ");
    TalonUtil.checkError(motor.config_kF(0, Constants.CLIMBER.F, Constants.CAN_TIMEOUT), "Climber: could not set F: ");

    StatorCurrentLimitConfiguration limitStator = new StatorCurrentLimitConfiguration(true, 60, 60, 0.2);
    TalonUtil.checkError(motor.configStatorCurrentLimit(limitStator, Constants.CAN_TIMEOUT), "Climber: Could not set stator current limits");
    TalonUtil.checkError(motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT), "Climber: Could not set voltage comp saturation");
    motor.enableVoltageCompensation(true);

    motor.setInverted(isInverted);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, Constants.CAN_TIMEOUT);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 60, Constants.CAN_TIMEOUT);
    motor.overrideLimitSwitchesEnable(true);
  }

  @Override
  public void cacheSensors() {
    cache.limitL = masterL.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cache.limitR = masterR.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cache.positionL = ticksToUnits(masterL.getSelectedSensorPosition());
    cache.positionR = ticksToUnits(masterR.getSelectedSensorPosition());
    resetIfAtHome();
  }

  @Override
  public void updateHardware() {
    updateSolenoids();
    updateMotors();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    if (showDetails) {
      SmartDashboard.putBoolean("Climber: Limit L", cache.limitL);
      SmartDashboard.putBoolean("Climber: Limit R", cache.limitR);
      if (Constants.TUNING.CLIMBER) {
        SmartDashboard.putNumber("Climber: Position L", cache.positionL);
        SmartDashboard.putNumber("Climber: Position R", cache.positionR);
        SmartDashboard.putNumber("Climber: Position Raw", masterL.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climber: Velocity Raw", masterL.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Climber: Current", masterL.getStatorCurrent());
      }
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    updateBrakeMode(true);
  }

  public void setOpenLoop(double percent) { outputs.set(TalonFXControlMode.PercentOutput, percent, percent); }
  public void setClosedLoopPosition(double percentUp) { outputs.set(TalonFXControlMode.Position, percentUp, percentUp); }
  public void setArms(boolean extend) { outputs.extend = extend; }
  public void setClimbMode(boolean enable) { inClimbMode = enable; }

  public void resetIfAtHome() {
    if (isAtHomingLocation(true)) {
      zeroSensors(true);
    }
    if (isAtHomingLocation(false)) {
      zeroSensors(false);
    }
  }

  public void zeroSensors(boolean left) {
    forceZero(left);
    if (left) {
      hasBeenZeroedL = true;
    } else {      
      hasBeenZeroedR = true;
    }
  }

  public void forceZero(boolean left) {
    TalonFX motor = (left) ? masterL : masterR;
    motor.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    if (left) {
      cache.positionL = 0.0;
    } else {
      cache.positionR = 0.0;
    }
  }

  public void setSoftLimitsEnabled(boolean enable) {
    masterL.overrideSoftLimitsEnable(enable);
    masterR.overrideSoftLimitsEnable(enable);
  }

  public double getPosition(boolean left) { return left ? cache.positionL : cache.positionR; }

  public boolean isExtended() { return cache.extended; }
  public boolean isInClimbMode() { return inClimbMode; }
  public boolean isZeroed() { return hasBeenZeroedL && hasBeenZeroedR; }
  public boolean isAtHomingLocation(boolean left) { return left ? cache.limitL : cache.limitR; }
  public boolean isStable() { return isOnTarget(true) && isOnTarget(false); }

  private double ticksToUnits(double ticks) { return ticks / Constants.CLIMBER.TICKS_PER_UNIT_DISTANCE; }
  private double unitsToTicks(double units) { return units * Constants.CLIMBER.TICKS_PER_UNIT_DISTANCE; }
  private boolean isOnTarget(boolean left) {
    double setpoint = left ? outputs.setpointL : outputs.setpointR;
    return Math.abs(getPosition(left) - setpoint) < Constants.CLIMBER.POSITION_DEADBAND;
  }

  private void updateMotors() {
    double setpointL = outputs.mode == TalonFXControlMode.Position ? unitsToTicks(outputs.setpointL) : outputs.setpointL;
    double setpointR = outputs.mode == TalonFXControlMode.Position ? unitsToTicks(outputs.setpointR) : outputs.setpointR;
    masterL.set(outputs.mode, setpointL, DemandType.ArbitraryFeedForward, 0.0);
    masterR.set(outputs.mode, setpointR, DemandType.ArbitraryFeedForward, 0.0);
  }

  private void updateSolenoids() {
    if (outputs.extend != cache.extended) {
      cache.extended = outputs.extend;
      Value direction = (cache.extended) ? Value.kForward : Value.kReverse;
      armL.set(direction);
      armR.set(direction);
    }
  }

  private void updateBrakeMode(boolean enable) {
    if (cache.inBrakeMode != enable) {
      cache.inBrakeMode = enable;
      NeutralMode mode = cache.inBrakeMode ? NeutralMode.Brake : NeutralMode.Coast;
      masterL.setNeutralMode(mode);
      masterR.setNeutralMode(mode);
    }
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkSolenoid(this, armL);
    Test.checkSolenoid(this, armR);
    Test.add(this, "Limit L", cache.limitL);
    Test.add(this, "Limit R", cache.limitR);
    Test.add(this, "Brake Mode", cache.inBrakeMode);
    Test.add(this, "Zeroed", isZeroed());
    Test.add(this, "Extended", isExtended());
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, masterR);
  }
}
