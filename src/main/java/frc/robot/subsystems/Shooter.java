package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team4646.StabilityCounter;
import frc.team4646.Test;

public class Shooter extends SmartSubsystem {
  private class DataCache {
    public double nativeVelocityL, nativeVelocityR;
    public double rpmL, rpmR;
  }

  private final TalonFX masterL, masterR;
  private final StabilityCounter stability = new StabilityCounter(Constants.SHOOTER.STABLE_COUNTS);
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;

  public Shooter() {
    masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_L);
    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_R);
    configureMotor(masterL, true);
    configureMotor(masterR, false);
  }

  public void configureMotor(TalonFX motor, boolean inverted) {
    motor.setInverted(inverted);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);

    TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Shooter: Could not detect encoder: ");
    TalonUtil.checkError(motor.config_kP(0, Constants.SHOOTER.P, Constants.CAN_TIMEOUT), "Shooter: could not set P: ");
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
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Shooter: Stable", stability.isStable());
    if (Constants.TUNING.SHOOTERS) {
      SmartDashboard.putNumber("Shooter: RPM", getRPM());
      SmartDashboard.putNumber("Shooter: Error", getErrorRPM());
      SmartDashboard.putNumber("Shooter: Demand", demand);
    }
  }

  public void setOpenLoop(double percent) {
    // targetVelocityRPM = Double.POSITIVE_INFINITY;
    masterL.set(TalonFXControlMode.PercentOutput, percent);
    masterR.set(TalonFXControlMode.PercentOutput, percent);
    demand = percent;
  }

  public void setClosedLoop(double rpm) {
    targetVelocityRPM = rpm;
    masterL.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    masterR.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    demand = rpm;
  }

  public double getRPM() { return (cache.rpmL + cache.rpmR) / 2.0; }
  public double getSetpoint() { return demand; }

  public boolean isShooting() { return demand >= Constants.VISION.shootTree.getRPMBottomMin() * 0.9; }
  public boolean isStable() { return stability.isStable(); }

  private double nativeUnitsToRPM(double ticks_per_100_ms) { return ticks_per_100_ms * 10.0 * 60.0 / Constants.SHOOTER.TICKS_PER_REV; }
  private double rpmToNativeUnits(double rpm) { return rpm / 60.0 / 10.0 * Constants.SHOOTER.TICKS_PER_REV; }
  private double getErrorRPM() { return Math.abs(targetVelocityRPM - getRPM()); }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, masterR);
  }
}
