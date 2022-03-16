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
import frc.team4646.Test;

public class ShooterTop extends SmartSubsystem {
  public static class DataCache {
    public double nativeVelocityL;
    public double rpmL;
    public double ampsStatorL;
    public double errorL;
  }

  private final TalonFX masterTop;
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  public ShooterTop() {
    masterTop = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_TOP);
    masterTop.setInverted(false);
    configureMotor(masterTop);
  }

  public void configureMotor(TalonFX motor) {
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);

    TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "ShooterTop: Could not detect encoder: ");
    TalonUtil.checkError(motor.config_kP(0, Constants.SHOOTER.P, Constants.CAN_TIMEOUT), "ShooterTop: could not set P: ");
    TalonUtil.checkError(motor.config_kF(0, Constants.SHOOTER.F, Constants.CAN_TIMEOUT), "ShooterTop: could not set F: ");

    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 30.0, 100.0, 0.02);
    TalonUtil.checkError(motor.configSupplyCurrentLimit(limit), "ShooterTop: Could not set supply current limit");
  }

  @Override
  public void cacheSensors() {
    cache.nativeVelocityL = masterTop.getSelectedSensorVelocity();
    cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    cache.ampsStatorL = masterTop.getStatorCurrent();
    cache.errorL = (masterTop.getControlMode() == ControlMode.Velocity) ? masterTop.getClosedLoopError(0) : 0.0;

    stableCounts++;
    if(Math.abs(targetVelocityRPM - getRPM()) > Constants.SHOOTER_TOP.RPM_ERROR_ALLOWED || !isShooting()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard(boolean showDetails) {    
    if (Constants.SHOOTER.TUNING) {
      SmartDashboard.putNumber("ShooterTop: RPM", getRPM());
      SmartDashboard.putNumber("ShooterTop: IsStable", stableCounts);
      SmartDashboard.putNumber("ShooterTop: Demand", demand);
      SmartDashboard.putNumber("ShooterTop: Amps Stator L", cache.ampsStatorL);
      SmartDashboard.putNumber("ShooterTop: Error L", cache.errorL);
    }
  }

  public void setOpenLoop(double percent) {
    masterTop.set(TalonFXControlMode.PercentOutput, percent);
    demand = percent;
  }

  public void setClosedLoop(double rpm) {
    targetVelocityRPM = rpm;
    masterTop.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    demand = rpm;
  }

  public double getAmpsStator() { return (cache.ampsStatorL); }
  public double getRPM() { return (cache.rpmL); }
  public double getSetpoint() { return demand; }

  public boolean isShooting() { return demand >= Constants.VISION.shootTree.getRPMTopMin() * 0.9; }
  public boolean isStable() { return stableCounts >= Constants.SHOOTER_TOP.STABLE_COUNTS; }

  private double nativeUnitsToRPM(double ticks_per_100_ms) { return ticks_per_100_ms * 10.0 * 60.0 / Constants.SHOOTER_TOP.TICKS_PER_REV; }
  private double rpmToNativeUnits(double rpm) { return rpm / 60.0 / 10.0 * Constants.SHOOTER_TOP.TICKS_PER_REV; }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterTop);
    Test.checkStatusFrames(this, masterTop);
  }
}
