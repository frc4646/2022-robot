package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.SparkMaxFactory;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team254.util.Util;

public class Shooter extends SmartSubsystem {
  public static class DataCache {
    public double voltsL, voltsR;
    public double nativeVelocityL, nativeVelocityR;
    public double rpmL, rpmR;
    public double ampsSupplyL, ampsSupplyR;
    public double ampsStatorL, ampsStatorR;
    public double errorL, errorR;
  }

  // private final CANSparkMax leftMaster;
  private final CANSparkMax rightMaster;
  // private final TalonFX masterL, masterR;
  private final DataCache cache = new DataCache();
  // private NetworkTableEntry guiRPM_L, guiRPM_R, graphRPM, graphAmps_L, graphAmps_R;

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  public Shooter() {
    // leftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.SHOOTER_L);
    // leftMaster.setInverted(true);
    // leftMaster.setIdleMode(IdleMode.kCoast);
    // leftMaster.enableVoltageCompensation(12.0);
    // leftMaster.getPIDController().setP(Constants.Shooter.P);
    // leftMaster.getPIDController().setI(Constants.Shooter.I);
    // leftMaster.getPIDController().setD(Constants.Shooter.D);
    // leftMaster.getPIDController().setFF(Constants.Shooter.F);

    rightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.SHOOTER_R);
    rightMaster.setInverted(false);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightMaster.enableVoltageCompensation(12.0);
    rightMaster.getPIDController().setP(Constants.Shooter.P);
    rightMaster.getPIDController().setI(Constants.Shooter.I);
    rightMaster.getPIDController().setD(Constants.Shooter.D);
    rightMaster.getPIDController().setFF(Constants.Shooter.F);

    // masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_L);
    // masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_R);
  
    // masterL.setInverted(true);
    // masterL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    // masterL.enableVoltageCompensation(true);

    // masterR.setInverted(false);
    // masterR.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    // masterR.enableVoltageCompensation(true);

    // TalonUtil.checkError(masterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Shooter MasterL: Could not detect encoder: ");
    // TalonUtil.checkError(masterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), "Shooter MasterR: Could not detect encoder: ");

    // TalonUtil.checkError(masterL.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT), "Shooter MasterL: could not set P: ");
    // TalonUtil.checkError(masterL.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT), "Shooter MasterL: could not set I: ");
    // TalonUtil.checkError(masterL.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT), "Shooter MasterL: could not set D: ");
    // TalonUtil.checkError(masterL.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT), "Shooter MasterL: could not set F: ");

    // TalonUtil.checkError(masterR.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT), "Shooter MasterR: could not set P: ");
    // TalonUtil.checkError(masterR.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT), "Shooter MasterR: could not set I: ");
    // TalonUtil.checkError(masterR.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT), "Shooter MasterR: could not set D: ");
    // TalonUtil.checkError(masterR.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT), "Shooter MasterR: could not set F: ");

    // TODO limit supply current?
    // SupplyCurrentLimitConfiguration current = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
    // masterL.configSupplyCurrentLimit(current);
    // masterR.configSupplyCurrentLimit(current);

    // ShuffleboardLayout layoutRpm = DashboardControls.addLayout("Shooter", "RPM").withSize(4, 8);
    // graphRPM = layoutRpm.add("Shooter RPM Avg", getRPM()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 0)).getEntry();
    // guiRPM_L = layoutRpm.add("Shooter RPM L", cache.rpmL).getEntry();
    // guiRPM_R = layoutRpm.add("Shooter RPM R", cache.rpmR).getEntry();

    // ShuffleboardLayout layoutAmps = DashboardControls.addLayout("Shooter", "Amps").withSize(4, 8);
    // graphAmps_L = layoutAmps.add("Shooter Amps L", cache.ampsStatorL).withWidget(BuiltInWidgets.kGraph).getEntry();
    // graphAmps_R = layoutAmps.add("Shooter Amps R", cache.ampsStatorR).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  @Override
  public void cacheSensors() {
    // cache.rpmL = leftMaster.getEncoder().getVelocity();
    // cache.ampsStatorL = leftMaster.getOutputCurrent();
    cache.rpmR = rightMaster.getEncoder().getVelocity();
    cache.ampsStatorR = rightMaster.getOutputCurrent();

    // cache.voltsL = masterL.getMotorOutputVoltage();
    // cache.voltsR = masterR.getMotorOutputVoltage();
    // cache.nativeVelocityL = masterL.getSelectedSensorVelocity();
    // cache.nativeVelocityR = masterR.getSelectedSensorVelocity();
    // cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    // cache.rpmR = nativeUnitsToRPM(cache.nativeVelocityR);
    // cache.ampsSupplyL = masterL.getSupplyCurrent();
    // cache.ampsSupplyR = masterR.getSupplyCurrent();
    // cache.ampsStatorL = masterL.getStatorCurrent();
    // cache.ampsStatorR = masterR.getStatorCurrent();
    // cache.errorL = (masterL.getControlMode() == ControlMode.Velocity) ? masterL.getClosedLoopError(0) : 0.0;
    // cache.errorR = (masterR.getControlMode() == ControlMode.Velocity) ? masterR.getClosedLoopError(0) : 0.0;
    stableCounts++;
    if (!isOnTarget()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard() {
    // guiRPM_L.setDouble(cache.rpmL);
    // guiRPM_R.setDouble(cache.rpmR);
    // graphRPM.setDouble(getRPM());
    // graphAmps_L.setDouble(cache.ampsStatorL);
    // graphAmps_R.setDouble(cache.ampsStatorR);

    SmartDashboard.putNumber("Shooter: RPM", getRPM());
    if (Constants.Shooter.TUNING) {
      // SmartDashboard.putNumber("Shooter: RPM L", cache.rpmL);
      // SmartDashboard.putNumber("Shooter: RPM R", cache.rpmR);
      SmartDashboard.putNumber("Shooter: Amps Supply L", cache.ampsSupplyL);
      SmartDashboard.putNumber("Shooter: Amps Supply R", cache.ampsSupplyR);
      SmartDashboard.putNumber("Shooter: Amps Stator L", cache.ampsStatorL);
      SmartDashboard.putNumber("Shooter: Amps Stator R", cache.ampsStatorR);
      SmartDashboard.putNumber("Shooter: Error L", cache.errorL);
      SmartDashboard.putNumber("Shooter: Error R", cache.errorR);
    }
  }

  public void setOpenLoop(double percent) {
    // leftMaster.set(percent);
    rightMaster.set(percent);
    // masterL.set(TalonFXControlMode.PercentOutput, percent);
    // masterR.set(TalonFXControlMode.PercentOutput, percent);
    demand = percent;
  }

  public void setClosedLoop(double rpm) {
    targetVelocityRPM = rpm;
    // leftMaster.getPIDController().setReference(rpm, ControlType.kVelocity);
    rightMaster.getPIDController().setReference(rpm, ControlType.kVelocity);
    // masterL.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    // masterR.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    demand = rpm;
  }

  public double getAmpsSupply() { return (cache.ampsSupplyL + cache.ampsSupplyR) / 2.0; }
  public double getAmpsStator() { return (cache.ampsStatorL + cache.ampsStatorR) / 2.0; }
  public double getRPM() { return (cache.rpmL + cache.rpmR) / 2.0; }
  public double getVoltage() { return (cache.voltsL + cache.voltsR) / 2.0; }

  public boolean isOnTarget() { return Math.abs(targetVelocityRPM - getRPM()) < Constants.Shooter.RPM_ERROR_ALLOWED; }
  public boolean isShooting() { return !Util.epsilonEquals(demand, 0.0); }
  public boolean isStable() { return stableCounts > Constants.Shooter.RPM_STABLE_COUNTS; }
  
  private double nativeUnitsToRPM(double ticks_per_100_ms) {
    return ticks_per_100_ms;  // sparkmax already rpm
    // return ticks_per_100_ms * 10.0 * 60.0 / Constants.Shooter.TICKS_PER_REV;
  }
  private double rpmToNativeUnits(double rpm) {
    return rpm;
    // return rpm / 60.0 / 10.0 * Constants.Shooter.TICKS_PER_REV;
  }

  @Override
  public void runTests() {
    // Test.checkFirmware(this, masterL);
    // Test.checkFirmware(this, masterR);
  }
}
