package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.ShootSetpoint;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team4646.PIDTuner;
import frc.team4646.StabilityCounter;
import frc.team4646.Test;

public class Shooter extends SmartSubsystem {
  private class DataCache {
    public double nativeVelocityL, nativeVelocityR;
    public double rpmL, rpmR;
  }  
  private class OutputCache {
    public TalonFXControlMode mode = TalonFXControlMode.PercentOutput;
    public double setpoint = 0.0;
    public boolean intendToShoot = false;

    public void set(TalonFXControlMode mode, double setpoint, boolean intendToShoot) {
      outputs.mode = mode;
      outputs.setpoint = setpoint;
      outputs.intendToShoot = intendToShoot;
    }
  }

  private final TalonFX masterL, masterR;
  private final StabilityCounter stability = new StabilityCounter(Constants.SHOOTER.STABLE_COUNTS);
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();
  private PIDTuner tuner;

  public Shooter() {
    masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_L);
    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_R);
    configureMotor(masterL, true);
    configureMotor(masterR, false);
    
    if(Constants.TUNING.SHOOTERS) {
      tuner = new PIDTuner("Shooter",Constants.SHOOTER.P,
      Constants.SHOOTER.I,
      Constants.SHOOTER.D,
      Constants.SHOOTER.F,
      Constants.SHOOTER.CRACKPOINT, masterL, masterR);
    }
  }

  public void configureMotor(TalonFX motor, boolean inverted) {
    motor.setInverted(inverted);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);

    TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Shooter: Could not detect encoder: ");
    TalonUtil.checkError(motor.config_kP(0, Constants.SHOOTER.P, Constants.CAN_TIMEOUT), "Shooter: could not set P: ");
    TalonUtil.checkError(motor.config_kI(0, Constants.SHOOTER.I, Constants.CAN_TIMEOUT), "Shooter: could not set I: ");
    TalonUtil.checkError(motor.config_kD(0, Constants.SHOOTER.D, Constants.CAN_TIMEOUT), "Shooter: could not set D: ");
    TalonUtil.checkError(motor.config_kF(0, Constants.SHOOTER.F, Constants.CAN_TIMEOUT), "Shooter: could not set F: ");

    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40.0, 100.0, 0.02);
    TalonUtil.checkError(motor.configSupplyCurrentLimit(limit), "Shooter: Could not set supply current limit");
  }

  @Override
  public void cacheSensors() {
    cache.nativeVelocityL = masterL.getSelectedSensorVelocity();
    cache.nativeVelocityR = masterR.getSelectedSensorVelocity();
    cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    cache.rpmR = nativeUnitsToRPM(cache.nativeVelocityR);
    stability.calculate(isShooting() && getErrorRPM() < Constants.SHOOTER.RPM_ERROR_ALLOWED);
  }

  @Override
  public void updateHardware() {
    updateMotors();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Shooter: Stable", stability.isStable());
    if (Constants.TUNING.SHOOTERS) {
      SmartDashboard.putNumber("Shooter: RPMCmd", outputs.setpoint);
      SmartDashboard.putNumber("Shooter: RPM", getRPM());
      SmartDashboard.putNumber("Shooter: Error", getErrorRPM());
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    
    if(Constants.TUNING.SHOOTERS) {
    tuner.updateMotorPIDF();
    }
  }
  public void setOpenLoop(double percent) { outputs.set(TalonFXControlMode.PercentOutput, percent, percent > 0.0); }
  public void setClosedLoop(ShootSetpoint setpoint, boolean isShooting) { outputs.set(TalonFXControlMode.Velocity, setpoint.rpmBottom, isShooting); }

  public double getRPM() { return (cache.rpmL + cache.rpmR) / 2.0; }

  public boolean isIntendingToShoot() { return outputs.intendToShoot; }
  public boolean isShooting() { return outputs.mode == TalonFXControlMode.Velocity && outputs.setpoint >= Constants.VISION.MAP.getRPMBottomMin() * 0.9; }
  public boolean isStable() { return stability.isStable(); }

  private double getErrorRPM() { return Math.abs(outputs.setpoint - getRPM()); }
  private double nativeUnitsToRPM(double ticks_per_100_ms) { return ticks_per_100_ms * 10.0 * 60.0 / Constants.SHOOTER.TICKS_PER_REV; }
  private double rpmToNativeUnits(double rpm) { return rpm / 60.0 / 10.0 * Constants.SHOOTER.TICKS_PER_REV; }

  private void updateMotors() {
    double setpoint = outputs.mode == TalonFXControlMode.Velocity ? rpmToNativeUnits(outputs.setpoint) : outputs.setpoint;
    double feedForward = setpoint > 0.0 ? Constants.SHOOTER.CRACKPOINT : 0.0;
    masterL.set(outputs.mode, setpoint, DemandType.ArbitraryFeedForward, feedForward);
    masterR.set(outputs.mode, setpoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, masterR);
  }
}
