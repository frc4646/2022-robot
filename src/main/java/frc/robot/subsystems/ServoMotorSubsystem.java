package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;
import frc.team254.util.Util;

/** Abstract base class for a subsystem with a single sensored servo-mechanism */
public abstract class ServoMotorSubsystem extends SmartSubsystem {
  private static final int kMotionProfileSlot = 0, kPositionPIDSlot = 1;

  /** Recommend initializing in a static block! */
  public static class TalonFXConstants {
    public int id = -1;
    public boolean invert_motor = false;
    public boolean invert_sensor_phase = false;
    public double encoder_ppr = 2048.0;
  }

  /** Recommend initializing in a static block! */
  public static class ServoMotorSubsystemConstants {
    public TalonFXConstants kMasterConstants = new TalonFXConstants();
    public TalonFXConstants[] kSlaveConstants = new TalonFXConstants[0];

    public double kHomePosition = 0.0; // Units
    public double kTicksPerUnitDistance = 1.0;
    public double kKp = 0.0;  // Raw output / raw error
    public double kKi = 0.0;  // Raw output / sum of raw error
    public double kKd = 0.0;  // Raw output / (err - prevErr)
    public double kKf = 0.0;  // Raw output / velocity in ticks/100ms
    public double kKa = 0.0;  // Raw output / accel in (ticks/100ms) / s
    public double kMaxIntegralAccumulator = 0.0;
    public double kIZone = 0.0; // Ticks
    public double kDeadband = 0.0; // Ticks

    public double kPositionKp = 0.0;
    public double kPositionKi = 0.0;
    public double kPositionKd = 0.0;
    public double kPositionKf = 0.0;
    public double kPositionMaxIntegralAccumulator = 0.0;
    public double kPositionIZone = 0.0; // Ticks
    public double kPositionDeadband = 0.0; // Ticks

    public double kCruiseVelocity = 0.0; // Ticks / 100ms
    public double kAcceleration = 0.0; // Ticks / 100ms / s
    public double kRampRate = 0.0; // s
    public double kMaxVoltage = 12.0;

    public double kSupplyContinuousCurrentLimit = 20.0; // amps
    public double kSupplyPeakCurrentLimit = 60.0; // amps
    public double kSupplyPeakCurrentDuration = 0.2; // seconds
    public boolean kEnableSupplyCurrentLimit = false;

    public double kStatorContinuousCurrentLimit = 20.0; // amps
    public double kStatorPeakCurrentLimit = 60.0; // amps
    public double kStatorPeakCurrentDuration = 0.2; // seconds
    public boolean kEnableStatorCurrentLimit = false;

    public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
    public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

    public int kStatusFrame8UpdateRate = 1000;
    public boolean kRecoverPositionOnReset = false;
  }

  protected final ServoMotorSubsystemConstants mConstants;
  protected final TalonFX mMaster;
  protected final TalonFX[] mSlaves;

  protected final double mForwardSoftLimitTicks, mReverseSoftLimitTicks;

  protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
    mConstants = constants;
    mMaster = TalonFXFactory.createDefaultTalon(mConstants.kMasterConstants.id);
    mSlaves = new TalonFX[mConstants.kSlaveConstants.length];

    TalonUtil.checkError(mMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not detect encoder: ");

    mForwardSoftLimitTicks = (mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    mReverseSoftLimitTicks = (mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    TalonUtil.checkError(mMaster.configForwardSoftLimitThreshold(mForwardSoftLimitTicks, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set forward soft limit: ");
    TalonUtil.checkError(mMaster.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not enable forward soft limit: ");
    TalonUtil.checkError(mMaster.configReverseSoftLimitThreshold(mReverseSoftLimitTicks, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set reverse soft limit: ");
    TalonUtil.checkError(mMaster.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not enable reverse soft limit: ");

    TalonUtil.checkError(mMaster.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set voltage compensation saturation: ");

    TalonUtil.checkError(mMaster.config_kP(kMotionProfileSlot, mConstants.kKp, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kP: ");
    TalonUtil.checkError(mMaster.config_kI(kMotionProfileSlot, mConstants.kKi, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kI: ");
    TalonUtil.checkError(mMaster.config_kD(kMotionProfileSlot, mConstants.kKd, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kD: ");
    TalonUtil.checkError(mMaster.config_kF(kMotionProfileSlot, mConstants.kKf, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set kF: ");
    TalonUtil.checkError(mMaster.configMaxIntegralAccumulator(kMotionProfileSlot, mConstants.kMaxIntegralAccumulator, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set max integral: ");
    TalonUtil.checkError(mMaster.config_IntegralZone(kMotionProfileSlot, mConstants.kIZone, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set i zone: ");
    TalonUtil.checkError(mMaster.configAllowableClosedloopError(kMotionProfileSlot, mConstants.kDeadband, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set deadband: ");
    TalonUtil.checkError(mMaster.configMotionCruiseVelocity(mConstants.kCruiseVelocity, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set cruise velocity: ");
    TalonUtil.checkError(mMaster.configMotionAcceleration(mConstants.kAcceleration, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set acceleration: ");

    TalonUtil.checkError(mMaster.config_kP(kPositionPIDSlot, mConstants.kPositionKp, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kP: ");
    TalonUtil.checkError(mMaster.config_kI(kPositionPIDSlot, mConstants.kPositionKi, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kI: ");
    TalonUtil.checkError(mMaster.config_kD(kPositionPIDSlot, mConstants.kPositionKd, Constants.CAN_TIMEOUT_LONG), getName() + ": could not set kD: ");
    TalonUtil.checkError(mMaster.config_kF(kPositionPIDSlot, mConstants.kPositionKf, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set kF: ");
    TalonUtil.checkError(mMaster.configMaxIntegralAccumulator(kPositionPIDSlot, mConstants.kPositionMaxIntegralAccumulator, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set max integral: ");
    TalonUtil.checkError(mMaster.config_IntegralZone(kPositionPIDSlot, mConstants.kPositionIZone, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set i zone: ");
    TalonUtil.checkError(mMaster.configAllowableClosedloopError(kPositionPIDSlot, mConstants.kPositionDeadband, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set deadband: ");

    TalonUtil.checkError(mMaster.configOpenloopRamp(mConstants.kRampRate, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set voltage ramp rate: ");
    TalonUtil.checkError(mMaster.configClosedloopRamp(mConstants.kRampRate, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set closed loop ramp rate: ");

    SupplyCurrentLimitConfiguration limitsSupply = new SupplyCurrentLimitConfiguration(mConstants.kEnableSupplyCurrentLimit, mConstants.kSupplyContinuousCurrentLimit, mConstants.kSupplyPeakCurrentLimit, mConstants.kSupplyPeakCurrentDuration);
    StatorCurrentLimitConfiguration limitsStator = new StatorCurrentLimitConfiguration(mConstants.kEnableStatorCurrentLimit, mConstants.kStatorContinuousCurrentLimit, mConstants.kStatorPeakCurrentLimit, mConstants.kStatorPeakCurrentDuration);
    TalonUtil.checkError(mMaster.configSupplyCurrentLimit(limitsSupply), getName() + ": Could not set supply current limit.");
    TalonUtil.checkError(mMaster.configStatorCurrentLimit(limitsStator), getName() + ": Could not set stator current limit.");

    mMaster.configVoltageMeasurementFilter(8);
    TalonUtil.checkError(mMaster.configVoltageCompSaturation(mConstants.kMaxVoltage, Constants.CAN_TIMEOUT_LONG), getName() + ": Could not set voltage comp saturation.");
    mMaster.enableVoltageCompensation(true);

    mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
    mMaster.setSensorPhase(mConstants.kMasterConstants.invert_sensor_phase);
    mMaster.setNeutralMode(NeutralMode.Brake);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, 20);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);
    mMaster.selectProfileSlot(kMotionProfileSlot, 0);  // Start with kMotionProfileSlot

    for (int i = 0; i < mSlaves.length; ++i) {
      mSlaves[i] = TalonFXFactory.createPermanentSlaveTalon(mConstants.kSlaveConstants[i].id, mConstants.kMasterConstants.id);
      mSlaves[i].setInverted(mConstants.kSlaveConstants[i].invert_motor);  // TODO change to new follow/oppose master invert type
      mSlaves[i].setNeutralMode(NeutralMode.Brake);
      mSlaves[i].follow(mMaster);
    }

    setOpenLoop(0.0);
    mMaster.set(ControlMode.PercentOutput, 0.0);  // Send a neutral command
  }

  public static class PeriodicIO {
    // INPUTS
    public double position_ticks;
    public double position_units;
    public double velocity_ticks_per_100ms;
    public double active_trajectory_position; // ticks
    public double active_trajectory_velocity; // ticks/100ms
    public double active_trajectory_acceleration; // ticks/100ms/s
    public double output_percent;
    public double output_voltage;
    public double master_current;
    public double error_ticks;
    public double encoder_wraps;
    public double absolute_pulse_offset = 0;
    public double absolute_pulse_position_modded;
    public boolean reset_occured;
    // OUTPUTS
    public double demand;
    public double feedforward;
  }
  protected enum ControlState {
    OPEN_LOOP, MOTION_MAGIC, POSITION_PID
  }

  protected PeriodicIO mPeriodicIO = new PeriodicIO();
  protected ControlState mControlState = ControlState.OPEN_LOOP;
  protected boolean mHasBeenZeroed = false;
  protected StickyFaults mFaults = new StickyFaults();

  @Override
  public void cacheSensors() {
    if (mMaster.hasResetOccurred()) {
      DriverStation.reportError(getName() + ": Talon Reset! ", false);
      mPeriodicIO.reset_occured = true;
      return;
    } 
    mPeriodicIO.reset_occured = false;

    mMaster.getStickyFaults(mFaults);
    if (mFaults.hasAnyFault()) {
      DriverStation.reportError(getName() + ": Talon Fault! " + mFaults.toString(), false);
      mMaster.clearStickyFaults(0);
    }
    if (mMaster.getControlMode() == ControlMode.MotionMagic) {
      mPeriodicIO.active_trajectory_position = mMaster.getActiveTrajectoryPosition();

      if (mPeriodicIO.active_trajectory_position < mReverseSoftLimitTicks) {
        DriverStation.reportError(getName() + ": Active trajectory past reverse soft limit!", false);
      } else if (mPeriodicIO.active_trajectory_position > mForwardSoftLimitTicks) {
        DriverStation.reportError(getName() + ": Active trajectory past forward soft limit!", false);
      }
      final double newVel = mMaster.getActiveTrajectoryVelocity();
      if (Util.epsilonEquals(newVel, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband)) || Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
        // Mechanism is ~constant velocity.
        mPeriodicIO.active_trajectory_acceleration = 0.0;
      } else {
        // Mechanism is accelerating.
        mPeriodicIO.active_trajectory_acceleration = Math.signum(newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
      }
      mPeriodicIO.active_trajectory_velocity = newVel;
    } else {
      mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
      mPeriodicIO.active_trajectory_velocity = 0;
      mPeriodicIO.active_trajectory_acceleration = 0.0;
    }
    mPeriodicIO.error_ticks = (mMaster.getControlMode() == ControlMode.Position) ? mMaster.getClosedLoopError(0) : 0.0;

    mPeriodicIO.master_current = mMaster.getStatorCurrent();
    mPeriodicIO.output_voltage = mMaster.getMotorOutputVoltage();
    mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
    mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
    mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
    mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);

    if (mConstants.kRecoverPositionOnReset) {
      mPeriodicIO.absolute_pulse_position_modded = mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
      if (mPeriodicIO.absolute_pulse_position_modded < 0.0) {
        mPeriodicIO.absolute_pulse_position_modded += mConstants.kMasterConstants.encoder_ppr;
      }

      double estimated_pulse_pos = ((mConstants.kMasterConstants.invert_sensor_phase ? -1.0 : 1.0) * mPeriodicIO.position_ticks) + mPeriodicIO.absolute_pulse_offset;
      double new_wraps = Math.floor(estimated_pulse_pos / ((double) mConstants.kMasterConstants.encoder_ppr));
      // Only set this when we are really sure its a valid change
      if (Math.abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1.0) {
        mPeriodicIO.encoder_wraps = new_wraps;
      }
    }
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber(getName() + ": Position (units)", mPeriodicIO.position_units);
    SmartDashboard.putBoolean(getName() + ": Homing Location", atHomingLocation());
  }

  // ------------------------------ SETTERS: SENSORS ------------------------------
  /** Hint: Return <b>false</b> when first writing subsystem */
  public abstract boolean atHomingLocation();

  public void resetIfAtHome() {
    if (atHomingLocation()) {
      zeroSensors();
    }
  }

  public void zeroSensors() {
    mMaster.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_LONG);
    mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
    mHasBeenZeroed = true;
  }

  public void forceZero() {
    mMaster.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_LONG);
    mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
  }

  // ------------------------------ SETTERS: RECOVERY ------------------------------
  public void recoverFromReset() {
    if (mPeriodicIO.reset_occured) {
      System.out.println(getName() + ": Master Talon reset occurred; resetting frame rates.");
      mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
      mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
      mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

      // Reset encoder position to estimated position from absolute encoder
      if (mConstants.kRecoverPositionOnReset) {
        mMaster.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants.CAN_TIMEOUT_LONG);
      }
    }
    for (TalonFX slave : mSlaves) {
      if (slave.hasResetOccurred()) {
        System.out.println(getName() + ": Slave Talon reset occurred");
      }
    }
  }

  // ------------------------------ SETTERS: MOTORS ------------------------------
  public void setSetpointMotionMagic(double units, double feedforward_v) {
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
    if (mControlState != ControlState.MOTION_MAGIC) {
      mMaster.selectProfileSlot(kMotionProfileSlot, 0);
      mControlState = ControlState.MOTION_MAGIC;
    }
    mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
  }

  public void setSetpointPositionPID(double units, double feedforward_v) {
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
    if (mControlState != ControlState.POSITION_PID) {
      mMaster.selectProfileSlot(kPositionPIDSlot, 0);
      mControlState = ControlState.POSITION_PID;
    }
    mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
  }

  public void setOpenLoop(double percentage) {
    mPeriodicIO.demand = percentage;
    if (mControlState != ControlState.OPEN_LOOP) {
      mControlState = ControlState.OPEN_LOOP;
    }
    mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
  }

  // ------------------------------ GETTERS ------------------------------
  public double estimateSensorPositionFromAbsolute() {
    double estimated_pulse_pos = (mPeriodicIO.encoder_wraps * mConstants.kMasterConstants.encoder_ppr) + mPeriodicIO.absolute_pulse_position_modded;
    double estimate_position_ticks = (mConstants.kMasterConstants.invert_sensor_phase ? -1.0 : 1.0) * (estimated_pulse_pos - mPeriodicIO.absolute_pulse_offset);
    return estimate_position_ticks;
  }

  public double getPredictedPositionUnits(double lookahead_secs) {
    if (mMaster.getControlMode() != ControlMode.MotionMagic) {
      return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position + lookahead_secs * mPeriodicIO.active_trajectory_velocity + 0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
      return Math.min(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    } 
    return Math.max(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
  }

  /** In "Units" */
  public double getPosition() {
    return ticksToHomedUnits(mPeriodicIO.position_ticks);
  }

  /** In "Units per second" */
  public double getVelocity() {
    return ticksToUnits(mPeriodicIO.velocity_ticks_per_100ms) * 10.0;
  }  
 
  public double getSetpoint() {
    return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID) ? ticksToUnits(mPeriodicIO.demand) : Double.NaN;
  }

  public double getSetpointHomed() {
    return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID) ? ticksToHomedUnits(mPeriodicIO.demand) : Double.NaN;
  }

  public double getPositionTicks() {
    return mPeriodicIO.position_ticks;
  }

  public boolean hasBeenZeroed() {
    return mHasBeenZeroed;
  }

  /** @return absolute encoders raw ticks bounded to one rotation */
  protected double getAbsoluteEncoderRawPosition() {
    double abs_raw_with_rollover = mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0.0 ? abs_raw_with_rollover + mConstants.kMasterConstants.encoder_ppr : 0.0);
  }

  // ------------------------------ UNIT CONVERSION ------------------------------
  protected double ticksToUnits(double ticks) {
    return ticks / mConstants.kTicksPerUnitDistance;
  }

  protected double ticksToHomedUnits(double ticks) {
    return ticksToUnits(ticks) + mConstants.kHomePosition;
  }

  protected double unitsToTicks(double units) {
    return units * mConstants.kTicksPerUnitDistance;
  }

  protected double homeAwareUnitsToTicks(double units) {
    return unitsToTicks(units - mConstants.kHomePosition);
  }

  protected double constrainTicks(double ticks) {
    return Util.limit(ticks, mReverseSoftLimitTicks, mForwardSoftLimitTicks);
  }

  protected double ticksPer100msToUnitsPerSecond(double ticks_per_100ms) {
    return ticksToUnits(ticks_per_100ms) * 10.0;
  }

  protected double unitsPerSecondToTicksPer100ms(double units_per_second) {
    return unitsToTicks(units_per_second) / 10.0;
  }
}