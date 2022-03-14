package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team254.util.Util;
import frc.team4646.Test;

public class Shooter extends SmartSubsystem {
  public static class DataCache {
    public double nativeVelocityL, nativeVelocityR;
    public double rpmL, rpmR;
    public double ampsStatorL, ampsStatorR;
    public double errorL, errorR;
  }

  private final TalonFX masterL, masterR;
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  public Shooter() {
    masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_L);
    masterL.setInverted(true);
    configureMotor(masterL);

    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_R);  
    masterR.setInverted(false);
    configureMotor(masterR);
  }

  public void configureMotor(TalonFX motor) {
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
    cache.ampsStatorL = masterL.getStatorCurrent();
    cache.ampsStatorR = masterR.getStatorCurrent();
    cache.errorL = (masterL.getControlMode() == ControlMode.Velocity) ? masterL.getClosedLoopError(0) : 0.0;
    cache.errorR = (masterR.getControlMode() == ControlMode.Velocity) ? masterR.getClosedLoopError(0) : 0.0;

    stableCounts++;
    if(Math.abs(targetVelocityRPM - getRPM()) > Constants.SHOOTER.RPM_ERROR_ALLOWED || !isShooting()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    if (Constants.SHOOTER.TUNING) {
      SmartDashboard.putNumber("Shooter: RPM", getRPM());
      SmartDashboard.putNumber("Shooter: IsStable", stableCounts);
      SmartDashboard.putNumber("Shooter: Demand", demand);
      SmartDashboard.putNumber("Shooter: Amps Stator L", cache.ampsStatorL);
      SmartDashboard.putNumber("Shooter: Amps Stator R", cache.ampsStatorR);
      SmartDashboard.putNumber("Shooter: Error L", cache.errorL);
      SmartDashboard.putNumber("Shooter: Error R", cache.errorR);
    }
  }

  public void setOpenLoop(double percent) {
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

  public double getAmpsStator() { return (cache.ampsStatorL + cache.ampsStatorR) / 2.0; }
  public double getRPM() { return (cache.rpmL + cache.rpmR) / 2.0; }
  public double getSetpoint() { return demand; }

  public boolean isShooting() { return demand >= Constants.VISION.RPM_USABLE_MIN * 0.9; }
  public boolean isStable() { return stableCounts >= Constants.SHOOTER.STABLE_COUNTS; }

  private double nativeUnitsToRPM(double ticks_per_100_ms) { return ticks_per_100_ms * 10.0 * 60.0 / Constants.SHOOTER.TICKS_PER_REV; }
  private double rpmToNativeUnits(double rpm) { return rpm / 60.0 / 10.0 * Constants.SHOOTER.TICKS_PER_REV; }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkStatusFrames(this, masterL);
    Test.checkStatusFrames(this, masterR);
  }
}
