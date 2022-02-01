package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SmartSubsystem {
  public static class DataCache {
    public double voltsL, voltsR;
    public double nativeVelocityL, nativeVelocityR;
    public double rpmL, rpmR;
    public double ampsSupplyL, ampsSupplyR;
    public double ampsStatorL, ampsStatorR;
  }

  private final CANSparkMax leftMaster, rightSlave;
  // private final TalonFX masterL, masterR;
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  public Shooter() {
    leftMaster = new CANSparkMax(Constants.Ports.SHOOTER_L, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.SHOOTER_R, MotorType.kBrushless);

    rightSlave.follow(leftMaster, true);
    leftMaster.setInverted(true);
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    leftMaster.enableVoltageCompensation(12.0);
    rightSlave.enableVoltageCompensation(12.0);

    // masterL = TalonFXFactory.createDefaultTalon(Constants.Ports.SHOOTER_L);
    // masterR = TalonFXFactory.createDefaultTalon(Constants.Ports.SHOOTER_R);

    // masterL.setInverted(true);
    // masterL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    // masterL.enableVoltageCompensation(true);

    // masterR.setInverted(false);
    // masterR.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    // masterR.enableVoltageCompensation(true);

    // masterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG);
    // masterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG);

    // masterL.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT_LONG);
    // masterL.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT_LONG);
    // masterL.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT_LONG);
    // masterL.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT_LONG);

    // masterR.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT_LONG);
    // masterR.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT_LONG);
    // masterR.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT_LONG);
    // masterR.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT_LONG);

    // TODO limit supply current?
  }

  @Override
  public void cacheSensors() {
    cache.rpmL = leftMaster.getEncoder().getVelocity();
    cache.rpmR = rightSlave.getEncoder().getVelocity();
    cache.ampsStatorL = leftMaster.getOutputCurrent();
    cache.ampsStatorR = rightSlave.getOutputCurrent();

    // cache.voltageL = masterL.getMotorOutputVoltage();
    // cache.voltageR = masterR.getMotorOutputVoltage();
    // cache.nativeVelocityL = masterL.getSelectedSensorVelocity(0);
    // cache.nativeVelocityR = masterR.getSelectedSensorVelocity(0);
    cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    cache.rpmR = nativeUnitsToRPM(cache.nativeVelocityR);
    // cache.ampsSupplyL = masterL.getSupplyCurrent();
    // cache.ampsSupplyR = masterR.getSupplyCurrent();
    // cache.ampsStatorL = masterL.getStatorCurrent();
    // cache.ampsStatorR = masterR.getStatorCurrent();
    stableCounts++;
    if (!isOnTarget()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard() {
    // SmartDashboard.putNumber("Shooter Native L", cache.nativeVelocityL);
    // SmartDashboard.putNumber("Shooter Native R", cache.nativeVelocityR);
    SmartDashboard.putNumber("Shooter RPM L", cache.rpmL);
    SmartDashboard.putNumber("Shooter RPM R", cache.rpmR);
    SmartDashboard.putNumber("Shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter Demand", demand);
    SmartDashboard.putNumber("Shooter Amps L", cache.ampsStatorL);
    SmartDashboard.putNumber("Shooter Amps R", cache.ampsStatorR);
    // SmartDashboard.putNumber("Shooter Amps Supply L", cache.ampsSupplyL);
    // SmartDashboard.putNumber("Shooter Amps Supply R", cache.ampsSupplyR);
    // SmartDashboard.putNumber("Shooter Amps Stator L", cache.ampsStatorL);
    // SmartDashboard.putNumber("Shooter Amps Stator R", cache.ampsStatorR);
  }

  public void setOpenLoop(double percent) {
    leftMaster.set(percent);
    demand = percent;
  }

  public void setClosedLoop(double rpm) {
    setTargetRPM(rpm);
    leftMaster.getPIDController().setReference(rpm, ControlType.kVelocity);
    // masterL.set(ControlMode.Velocity, rpmToNativeUnits(rpm));
    // masterR.set(ControlMode.Velocity, rpmToNativeUnits(rpm));
    demand = rpm;
  }

  public void setTargetRPM(double rpm) {
    targetVelocityRPM = rpm;
  }

  public double getAmpsSupply() {
    return (cache.ampsSupplyL + cache.ampsSupplyR) / 2.0;
  }

  public double getAmpsStator() {
    return (cache.ampsStatorL + cache.ampsStatorR) / 2.0;
  }

  public double getRPM() {
    return (cache.rpmL + cache.rpmR) / 2.0;
  }

  public double getVoltage() {
    return (cache.voltsL + cache.voltsR) / 2.0;
  }

  public boolean isOnTarget() {
    return Math.abs(targetVelocityRPM - getRPM()) < Constants.Shooter.ERROR_ALLOWED_RPM;
  }

  public boolean isStable() {
    return stableCounts > 5;
  }

  private double nativeUnitsToRPM(double ticks_per_100_ms) {
    return ticks_per_100_ms * 10.0 * 60.0 / Constants.Shooter.TICKS_PER_REV;
  }

  // private double rpmToNativeUnits(double rpm) {
  //   return rpm / 60.0 / 10.0 * Constants.Shooter.TICKS_PER_REV;
  // }
}
