package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

public class ShooterTop extends SmartSubsystem {
  private class DataCache {
    public double nativeVelocityL;
    public double rpmL;
  }
  private class OutputCache {
    public TalonFXControlMode mode = TalonFXControlMode.PercentOutput;
    public double setpoint = 0.0;

    public void set(TalonFXControlMode mode, double setpoint) {
      outputs.mode = mode;
      outputs.setpoint = setpoint;
    }
  }

  private final TalonFX masterTop;
  private final StabilityCounter stability = new StabilityCounter(Constants.SHOOTER_TOP.STABLE_COUNTS);
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();
  private PIDTuner tuner;

  public ShooterTop() {
    masterTop = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_TOP);
    configureMotor(masterTop, false);
    
    if(Constants.TUNING.SHOOTERS) {
      tuner = new PIDTuner("ShooterTop",Constants.SHOOTER_TOP.PID, Constants.SHOOTER_TOP.CRACKPOINT, masterTop);
    }
  }

  public void configureMotor(TalonFX motor, boolean inverted) {
    motor.setInverted(inverted);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);

    TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "ShooterTop: Could not detect encoder: ");
    TalonFXFactory.setPID(motor, Constants.SHOOTER_TOP.PID);

    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 30.0, 100.0, 0.02);
    TalonUtil.checkError(motor.configSupplyCurrentLimit(limit), "ShooterTop: Could not set supply current limit");

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.CAN_TIMEOUT);
  }

  @Override
  public void cacheSensors() {
    cache.nativeVelocityL = masterTop.getSelectedSensorVelocity();
    cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    stability.calculate(isShooting() && getErrorRPM() < Constants.SHOOTER_TOP.RPM_ERROR_ALLOWED);
  }  

  @Override
  public void updateHardware() {
    updateMotors();
  }

  @Override
  public void updateDashboard(boolean showDetails) {   
    SmartDashboard.putBoolean("ShooterTop: Stable", stability.isStable()); 
    if (Constants.TUNING.SHOOTERS) {
      SmartDashboard.putNumber("ShooterTop: RPMCmd", outputs.setpoint);
      SmartDashboard.putNumber("ShooterTop: RPM", getRPM());
      SmartDashboard.putNumber("ShooterTop: Error", getErrorRPM());
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    
    if(Constants.TUNING.SHOOTERS) {
    tuner.updateMotorPIDF();
    }
  }

  public void setOpenLoop(double percent) { outputs.set(TalonFXControlMode.PercentOutput, percent); }
  public void setClosedLoop(ShootSetpoint setpoint) { outputs.set(TalonFXControlMode.Velocity, setpoint.rpmTop); }

  public double getRPM() { return (cache.rpmL); }

  public boolean isShooting() { return outputs.mode == TalonFXControlMode.Velocity && outputs.setpoint >= Constants.VISION.MAP.getRPMTopMin() * 0.9; }
  public boolean isStable() { return stability.isStable(); }

  private double getErrorRPM() { return Math.abs(outputs.setpoint - getRPM()); }
  private double nativeUnitsToRPM(double ticks_per_100_ms) { return ticks_per_100_ms * 10.0 * 60.0 / Constants.SHOOTER_TOP.TICKS_PER_REV; }
  private double rpmToNativeUnits(double rpm) { return rpm / 60.0 / 10.0 * Constants.SHOOTER_TOP.TICKS_PER_REV; }

  private void updateMotors() {
    double setpoint = outputs.mode == TalonFXControlMode.Velocity ? rpmToNativeUnits(outputs.setpoint) : outputs.setpoint;
    double feedForward = setpoint > 0.0 ? Constants.SHOOTER_TOP.CRACKPOINT : 0.0;
    masterTop.set(outputs.mode, setpoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterTop);
    Test.checkStatusFrames(this, masterTop);
  }
}
