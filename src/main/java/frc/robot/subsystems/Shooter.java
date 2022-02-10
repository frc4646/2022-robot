package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.controls.DashboardControls;
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
  }

  private final CANSparkMax leftMaster, rightSlave;
  private final TalonFX /* masterL,*/ masterR;
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  private NetworkTableEntry guiRPM_L;
  private NetworkTableEntry guiRPM_R;
  private NetworkTableEntry graphRPM;
  private NetworkTableEntry graphDemand;
  private NetworkTableEntry graphAmps_L;
  private NetworkTableEntry graphAmps_R;

  public Shooter() {
    leftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.SHOOTER_L);
    rightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.SHOOTER_R, leftMaster, true);

    leftMaster.setInverted(true);
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    leftMaster.enableVoltageCompensation(12.0);
    rightSlave.enableVoltageCompensation(12.0);

    leftMaster.getPIDController().setP(Constants.Shooter.P);
    leftMaster.getPIDController().setI(Constants.Shooter.I);
    leftMaster.getPIDController().setD(Constants.Shooter.D);
    leftMaster.getPIDController().setFF(Constants.Shooter.F);

    // masterL = TalonFXFactory.createDefaultTalon(Constants.Ports.SHOOTER_L);
    masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.TALON_SHOOTER_R);
  
    // masterL.setInverted(true);
    // masterL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    // masterL.enableVoltageCompensation(true);

    masterR.setInverted(false);
    masterR.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    masterR.enableVoltageCompensation(true);

    // TalonUtil.checkError(masterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: Could not detect encoder: ");
    TalonUtil.checkError(masterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: Could not detect encoder: ");

    // TalonUtil.checkError(masterL.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: could not set P: ");
    // TalonUtil.checkError(masterL.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: could not set I: ");
    // TalonUtil.checkError(masterL.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: could not set D: ");
    // TalonUtil.checkError(masterL.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT_LONG), "Shooter MasterL: could not set F: ");

    TalonUtil.checkError(masterR.config_kP(0, Constants.Shooter.P, Constants.CAN_TIMEOUT_LONG), "Shooter MasterR: could not set P: ");
    TalonUtil.checkError(masterR.config_kI(0, Constants.Shooter.I, Constants.CAN_TIMEOUT_LONG), "Shooter MasterR: could not set I: ");
    TalonUtil.checkError(masterR.config_kD(0, Constants.Shooter.D, Constants.CAN_TIMEOUT_LONG), "Shooter MasterR: could not set D: ");
    TalonUtil.checkError(masterR.config_kF(0, Constants.Shooter.F, Constants.CAN_TIMEOUT_LONG), "Shooter MasterR: could not set F: ");

    // TODO limit supply current?
    // SupplyCurrentLimitConfiguration current = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
    // masterL.configSupplyCurrentLimit(current);
    // masterR.configSupplyCurrentLimit(current);

    ShuffleboardLayout layoutRpm = DashboardControls.addLayout("Shooter", "RPM").withSize(4, 8);
    graphRPM = layoutRpm.add("Shooter RPM Avg", getRPM()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 0)).getEntry();
    graphDemand = layoutRpm.add("Shooter RPM Demand", demand).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 0)).getEntry();
    guiRPM_L = layoutRpm.add("Shooter RPM L", cache.rpmL).getEntry();
    guiRPM_R = layoutRpm.add("Shooter RPM R", cache.rpmR).getEntry();

    ShuffleboardLayout layoutAmps = DashboardControls.addLayout("Shooter", "Amps").withSize(4, 8);
    graphAmps_L = layoutAmps.add("Shooter Amps L", cache.ampsStatorL).withWidget(BuiltInWidgets.kGraph).getEntry();
    graphAmps_R = layoutAmps.add("Shooter Amps R", cache.ampsStatorR).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  @Override
  public void cacheSensors() {
    // cache.rpmL = leftMaster.getEncoder().getVelocity();
    // cache.rpmR = rightSlave.getEncoder().getVelocity();
    // cache.ampsStatorL = leftMaster.getOutputCurrent();
    // cache.ampsStatorR = rightSlave.getOutputCurrent();

    // cache.voltageL = masterL.getMotorOutputVoltage();
    cache.voltsR = masterR.getMotorOutputVoltage();
    // cache.nativeVelocityL = masterL.getSelectedSensorVelocity();
    cache.nativeVelocityR = masterR.getSelectedSensorVelocity();
    // cache.rpmL = nativeUnitsToRPM(cache.nativeVelocityL);
    cache.rpmR = nativeUnitsToRPM(cache.nativeVelocityR);
    // cache.ampsSupplyL = masterL.getSupplyCurrent();
    cache.ampsSupplyR = masterR.getSupplyCurrent();
    // cache.ampsStatorL = masterL.getStatorCurrent();
    cache.ampsStatorR = masterR.getStatorCurrent();
    stableCounts++;
    if (!isOnTarget()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard() {
    guiRPM_L.setDouble(cache.rpmL);
    guiRPM_R.setDouble(cache.rpmR);
    graphRPM.setDouble(getRPM());
    graphDemand.setDouble(demand);
    graphAmps_L.setDouble(cache.ampsStatorL);
    graphAmps_R.setDouble(cache.ampsStatorR);

    // SmartDashboard.putNumber("Shooter Native L", cache.nativeVelocityL);
    // SmartDashboard.putNumber("Shooter Native R", cache.nativeVelocityR);

    SmartDashboard.putNumber("Shooter RPM L", cache.rpmL);
    SmartDashboard.putNumber("Shooter RPM R", cache.rpmR);
    SmartDashboard.putNumber("Shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter Demand", demand);
    SmartDashboard.putNumber("Shooter Amps L", cache.ampsStatorL);
    SmartDashboard.putNumber("Shooter Amps R", cache.ampsStatorR);
    SmartDashboard.putNumber("Shooter Amps Supply L", cache.ampsSupplyL);
    SmartDashboard.putNumber("Shooter Amps Supply R", cache.ampsSupplyR);
    SmartDashboard.putNumber("Shooter Amps Stator L", cache.ampsStatorL);
    SmartDashboard.putNumber("Shooter Amps Stator R", cache.ampsStatorR);
  }

  public void setOpenLoop(double percent) {
    // leftMaster.set(percent);
    masterR.set( TalonFXControlMode.PercentOutput, 1.0);
    demand = percent;
  }

  public void setClosedLoop(double rpm) {
    setTargetRPM(rpm);
    // leftMaster.getPIDController().setReference(rpm, ControlType.kVelocity);
    // masterL.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
    masterR.set(TalonFXControlMode.Velocity, rpmToNativeUnits(rpm));
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
    return Math.abs(targetVelocityRPM - getRPM()) < Constants.Shooter.RPM_ERROR_ALLOWED;
  }

  public boolean isShooting() {
    return !Util.epsilonEquals(demand, 0.0);
  }

  public boolean isStable() {
    return stableCounts > 5;
  }

  private double nativeUnitsToRPM(double ticks_per_100_ms) {
    // return ticks_per_100_ms;  // sparkmax already rpm
    return ticks_per_100_ms * 10.0 * 60.0 / Constants.Shooter.TICKS_PER_REV;
  }

  private double rpmToNativeUnits(double rpm) {
    return rpm / 60.0 / 10.0 * Constants.Shooter.TICKS_PER_REV;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareSparkMax(this, leftMaster));
    Test.checkFirmware(new Test.FirmwareSparkMax(this, rightSlave));
    // TODO falcon firmware

    List<Integer> ids = new ArrayList<Integer>();
    List<DoubleSupplier> encoders = new ArrayList<DoubleSupplier>();
    ids.add(leftMaster.getDeviceId());
    ids.add(rightSlave.getDeviceId());
    encoders.add(() -> leftMaster.getEncoder().getPosition());
    encoders.add(() -> rightSlave.getEncoder().getPosition());
    testEncoder(ids, encoders);
  }

  // TODO refactor out to common class
  private boolean testEncoder(List<Integer> ids, List<DoubleSupplier> encoders) {
    List<Double> positionsInitial = new ArrayList<Double>();
    List<Double> positionsFinal = new ArrayList<Double>();
    boolean result = true;
    for (DoubleSupplier encoder : encoders) {
      positionsInitial.add(encoder.getAsDouble());
    }
    setOpenLoop(0.05);
    Timer.delay(0.1);
    setOpenLoop(0.0);
    for (DoubleSupplier encoder : encoders) {
      positionsFinal.add(encoder.getAsDouble());
    }
    Iterator<Integer> iterIds = ids.iterator();
    Iterator<Double> iterPositionsInitial = positionsInitial.iterator();
    Iterator<Double> iterPositionsFinal = positionsFinal.iterator();
    while (iterIds.hasNext() && iterPositionsInitial.hasNext() && iterPositionsFinal.hasNext()) {
      int id = iterIds.next();
      double delta = iterPositionsFinal.next() - iterPositionsInitial.next();
      if (delta < 0) {
        System.out.println(String.format("ERROR: Motor %d out of phase, moved %d ticks", id, delta));
        result = false;
      }
    }
    return result;
  }
}
