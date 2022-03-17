package frc.robot;

import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.robot.util.DiagnosticState;
import frc.robot.util.ShootTree;
import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;
import frc.team4646.LEDColor;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static int CAN_TIMEOUT = 100;

  public static final String SHOW_DETAILS = "Show Details";

  
  static {
    SmartDashboard.putBoolean(SHOW_DETAILS, false);
  }

  public static final class CAN {
    public static final int
      PNEUMATIC_CONTROL_MODULE = 0, POWER_DISTRIBUTION_PANEL = 2,
      DRIVETRAIN_FL = 24, DRIVETRAIN_BL = 23, DRIVETRAIN_BR = 22, DRIVETRAIN_FR = 21,
      TALON_SHOOTER_L = 25, TALON_SHOOTER_R = 27, TALON_SHOOTER_TOP = 28,
      CLIMBER_L = 13, CLIMBER_R = 14,
      INTAKE = 5, TURRET = 26,
      FEEDER = 8, AGITATOR_R = 31, AGITATOR_L = 32,
      CANIFIER = 9, CANDLE = 15;
  }

  public static final class SOLENOID {
    public static final int
      INTAKE_OUT = 1, INTAKE_IN = 0,
      ARM_L_OUT = 5, ARM_L_IN = 4, ARM_R_OUT = 7, ARM_R_IN = 6;
  }

  public static final class DIGITAL {
    public static final int
      FEEDER_BREAK_BEAM = 0;
  }

  public static final class AGITATOR {
    public static final double
      OPEN_LOOP_EXHAUST = 0.90,
      OPEN_LOOP_LOAD = 0.45,
      OPEN_LOOP_SHOOT = 0.45,
      OPEN_LOOP_RAMP = 0.125,  // TODO tune
      TIMEOUT_STOW = 0.5;  // TODO tune
  }
  
  public static final class CLIMBER {
    public static final boolean TUNING = false;

    public static final double
      OPEN_LOOP_ZERO = 0.1,
      DEADBAND = 0.2,
      TIMEOUT_ZERO = 5.0,
      GEAR_RATIO = 72.0 / 14.0,  // TODO correct values
      TICKS_PER_UNIT_DISTANCE = 2048.0 * GEAR_RATIO,
      LIMIT_F = 999999.0,
      POSITION_DEADBAND = 0.1,  // Tune
      P = 0.0,
      I = 0.0,
      D = 0.0,
      F = 0.0;
  }  

  public static final class COLORSENSOR {
    public static final boolean TUNING = false;
    
    public static final I2C.Port I2C_PORT = I2C.Port.kMXP;

    public static final int DISTANCE_MIN = 80;
    public static final Color 
      MATCH_BLUE = new Color(0.214, 0.325, 0.46),
      MATCH_RED = new Color(0.479, 0.378, 0.114);
  }

  public static final class DIAGNOSTICS {
    public static final double RUMBLE_PERCENT = 0.2;  // TODO tune
    public static final DiagnosticState
      FAULT_CARGO = new DiagnosticState(new LEDColor(1.0, 0.0, 1.0)),
      FAULT_TURRET = new DiagnosticState(new LEDColor(1.0, 1.0, 0.0), true),
      FAULT_OUTSIDE_VISION_RANGE = new DiagnosticState(new LEDColor(0.6, 0.0, 0.0)),
      CLIMBING = new DiagnosticState(new LEDColor(0.0, 1.0, 1.0)),
      CARGO_LOADED = new DiagnosticState(new LEDColor(0.3, 0.0, 0.3)),
      CAN_PRESS_SHOOT = new DiagnosticState(new LEDColor(0.0, 1.0, 0.0));
  }

  public static final class DRIVETRAIN {
    public static final boolean TUNING = false;

    public static final double
      TIMEOUT_DISABLED_COAST = 5.0,
      THROTTLE_SLEW_LIMIT = 1.20,  // % output per second
      THROTTLE_DEADBAND = 0.04,
      TURNING_DEADBAND = 0.035;

    public static final double VOLTAGE_COMPENSATION = 12.0;  // TODO INCREASE BEFORE COMPETITION
    public static final int CURRENT_LIMIT = 30;

    public static final double 
      WHEEL_DIAMETER = 6.0,
      WHEEL_TRACK_WIDTH_INCHES = 26.0,
      WHEEL_SCRUB_FACTOR = 1.02,
      GEAR_RATIO = 10.71,

      FEED_FORWARD_GAIN_STATIC = 0.31492,  // Tuned 3/6
      FEED_FORWARD_GAIN_VELOCITY = 2.87,
      FEED_FORWARD_GAIN_ACCEL = 0.81851,
      VOLTAGE_P = 0.001,  // Tuned 3/6
      VOLTAGE_I = 0.0,
      VOLTAGE_D = 0.0,
      VELOCITY_L_P = 0.0,
      VELOCITY_L_I = 0.0,
      VELOCITY_L_D = 0.0,
      VELOCITY_L_F = 0.0,
      VELOCITY_R_P = 0.0,
      VELOCITY_R_I = 0.0,
      VELOCITY_R_D = 0.0,
      VELOCITY_R_F = 0.0;
      //Left and right should be tuned via a step response to a velocity change - P = V, D = A, I = dist. Start

    public static final boolean IS_LEFT_ENCODER_INVERTED = false;
    public static final boolean IS_RIGHT_ENCODER_INVERTED = true;

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(WHEEL_TRACK_WIDTH_INCHES * 0.0254);
    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
      FEED_FORWARD_GAIN_STATIC, FEED_FORWARD_GAIN_VELOCITY, FEED_FORWARD_GAIN_ACCEL
    );

    public static final double
      AUTO_MAX_VOLTS = 10.0,
      MAX_SPEED_METERS_PER_SECOND = 3.6,
      MAX_ACCEL_METERS_PER_SECOND_SQUARED = 1.5,
      MAX_SPEED_METERS_PER_SECOND_SLOW = 2.0,
      MAX_ACCEL_METERS_PER_SECOND_SQUARED_SLOW = 1.0,
      RAMSETE_B = 1.0,     // FRC recommends 2.0
      RAMSETE_ZETA = 0.5;  // FRC recommends 0.7
      
    public static final DifferentialDriveVoltageConstraint
      AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, AUTO_MAX_VOLTS);
    public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCEL_METERS_PER_SECOND_SQUARED)
      .setKinematics(DRIVE_KINEMATICS)  // Ensures max speed is actually obeyed
      .addConstraint(AUTO_VOLTAGE_CONSTRAINT);
    public static final TrajectoryConfig TRAJECTORY_CONFIG_REVERSED = new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCEL_METERS_PER_SECOND_SQUARED)
      .setKinematics(DRIVE_KINEMATICS)  // Ensures max speed is actually obeyed
      .addConstraint(AUTO_VOLTAGE_CONSTRAINT)
      .setReversed(true);
      
    public static final TrajectoryConfig TRAJECTORY_CONFIG_SLOW = new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND_SLOW, MAX_ACCEL_METERS_PER_SECOND_SQUARED_SLOW)
    .setKinematics(DRIVE_KINEMATICS)  // Ensures max speed is actually obeyed
    .addConstraint(AUTO_VOLTAGE_CONSTRAINT);
    public static final TrajectoryConfig TRAJECTORY_CONFIG_SLOW_REVERSED = new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND_SLOW, MAX_ACCEL_METERS_PER_SECOND_SQUARED_SLOW)
    .setKinematics(DRIVE_KINEMATICS)  // Ensures max speed is actually obeyed
    .addConstraint(AUTO_VOLTAGE_CONSTRAINT)
    .setReversed(true);
  }

  public static final class FEEDER {
    public static final boolean TUNING = false;
    
    public static final double
      OPEN_LOOP_EXHAUST = 0.2,
      OPEN_LOOP_LOAD = 0.3,
      OPEN_LOOP_SHOOT = 0.5,
      OPEN_LOOP_RAMP = 0.25,  // TODO tune
      TIMEOUT_EXHAUST = 1.0,
      TIMEOUT_LOAD = 3.0,
      GEAR_RATIO = 72.0 / 14.0,
      POSITION_DEADBAND = 0.1,
      P = 0.1,
      I = 0.0,
      D = 0.0;
  }

  public static final class INFRASTRUCTURE {
    public static boolean CAMERA_STREAM = false;
  }

  public static final class INTAKE {
    public static final double
      OPEN_LOOP = 0.32846,
      OPEN_LOOP_RAMP = 0.4;
  }

  public static final class SHOOTER {
    public static final boolean TUNING = true;

    public static int STABLE_COUNTS = 2;
    public static final double
      OPEN_LOOP_REV_SECONDS = 1.0,
      RPM_MAX = 6380.0 * 1.105,  //  Tuned 3/15
      RPM_DEFAULT = 2200.0,
      RPM_ERROR_ALLOWED = 80.0,  // Tuned 3/1, 25-50 seem to work well TODO try 30 again
      RPM_TRIM = 150.0,
      DEADBAND = 0.05,
      TICKS_PER_REV = 2048.0,
      P = 0.01,  // Probably between 0.0075 and 0.25
      F =  TICKS_PER_REV / RPM_MAX / 60.0 * 10.0,
      WHEEL_DIAMETER = 4.0,
      GEAR_RATIO = 1.0 / 1.0;
  }

  public static final class SHOOTER_TOP {
    public static final boolean TUNING = true;

    public static int STABLE_COUNTS = SHOOTER.STABLE_COUNTS;
    public static final double
      RPM_MAX = 6380.0 * 1.25,  //  Tuned 3/15
      RPM_DEFAULT = 2200.0,
      RPM_ERROR_ALLOWED = 60.0,  // Tuned 3/1, 25-50 seem to work well TODO try 30 again
      RPM_TRIM = SHOOTER.RPM_TRIM * 2.0,
      TICKS_PER_REV = 2048.0,
      P = 0.02,  // Probably between 0.0075 and 0.25
      F =  TICKS_PER_REV / RPM_MAX / 60.0 * 10.0,
      WHEEL_DIAMETER = 2.0,
      GEAR_RATIO = 36.0 / 55.0;
  }

  public static final class TURRET {
    public static final boolean TUNING = false;
    public static final boolean SOFT_LIMITS_AT_STARTUP = true;

    public static final double
      STICK_GAIN = 25.0,
      STICK_DEADBAND = 0.4,  // TODO Reduce when switch to new controller
      GEAR_RATIO = 72.0 / 14.0 * 154.0 / 16.0,  // Number > 1 means "geared down"
      VELOCITY_MAX = 21500.0;

    public static final ServoMotorSubsystemConstants SERVO = new ServoMotorSubsystemConstants();
    static {
      SERVO.kMasterConstants.id = CAN.TURRET;
      SERVO.kMasterConstants.invert_motor = false;
      SERVO.kMasterConstants.invert_sensor_phase = false;
      SERVO.kSupplyContinuousCurrentLimit = 20;
      SERVO.kSupplyPeakCurrentLimit = 40;
      SERVO.kSupplyPeakCurrentDuration = 10; // ms

      SERVO.kMinUnitsLimit = 180.0 - 220.0;
      SERVO.kMaxUnitsLimit = 180.0 + 80.0;
      SERVO.kHomePosition = 180.0;
      SERVO.kTicksPerUnitDistance = 2048.0 * GEAR_RATIO / 360.0;

      SERVO.kPositionKp = 0.02;
      SERVO.kPositionKi = 0.0008;
      SERVO.kPositionKd = 0.25;
      SERVO.kPositionKf = 0.0;
      SERVO.kPositionIZone = 40.0;
      SERVO.kPositionMaxIntegralAccumulator = 20000.0;
      SERVO.kPositionDeadband = 0.1 * SERVO.kTicksPerUnitDistance; // Ticks

      SERVO.kMotionMagicKp = 0.6;
      SERVO.kMotionMagicKi = 0.0;
      SERVO.kMotionMagicKd = 1.2;
      SERVO.kMotionMagicKf = 1023.0 / VELOCITY_MAX;
      SERVO.kMotionMagicKa = 0.0;
      SERVO.kCruiseVelocity = VELOCITY_MAX * 0.95;
      SERVO.kAcceleration = SERVO.kCruiseVelocity * 1.0;
      SERVO.kMotionMagicDeadband = 1.0 * SERVO.kTicksPerUnitDistance; // Ticks

      // TODO SERVO.kRecoverPositionOnReset = true;
    }
  }

  public static final class VISION {
    public static final boolean TUNING = false;

    public static int STABLE_COUNTS = 2;

    public static final double
      HORIZONTAL_FOV = 54.0,  // Degrees (LL1: 54.0, LL2: 59.6)
      VERTICAL_FOV = 41.0,  // Degrees (LL1: 41.0, LL2: 49.7)
      CAMERA_MOUNTING_HEIGHT = 44.5,  // Inches
      CAMERA_MOUNTING_ANGLE = 33.2, // Degrees, tuned 2/27
      CAMERA_MOUNTING_OFFSET = 16.0;  // Tuned 3/4

    public static ShootTree shootTree = new ShootTree();
    static {
      shootTree.add(75.0, 1450.0, 2800.0);
      shootTree.add(90.0, 1500.0, 3000.0);
      shootTree.add(120.0, 1600.0, 3200.0);
      shootTree.add(150.0, 1700.0, 3400.0);
      shootTree.add(170.0, 1850.0, 3700.0);
    }
  }

  public static final class FIELD {
    public static final double CLIMBER_TIME_REQUIRED_TO_HOLD = 5.0;
    public static final double VISION_TAPE_INCHES = 102.0;
  }

  /** TRUE if not connected to the field at competition OR if the Smartdashboard button is pressed */
  public static boolean DashboardDuringComp() {
    return !DriverStation.isFMSAttached() || SmartDashboard.getBoolean(SHOW_DETAILS, false);
  }
}
