package frc.robot;

import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.robot.util.DiagnosticState;
import frc.robot.util.ShootMap;
import frc.team4646.LEDColor;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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
      AGITATOR_R = 31, AGITATOR_L = 32,
      INTAKE = 5, TURRET = 26, FEEDER = 8, CANIFIER = 9;
  }

  public static final class SOLENOID {
    public static final int
      INTAKE_OUT = 1, INTAKE_IN = 0,
      ARM_L_OUT = 5, ARM_L_IN = 4, ARM_R_OUT = 7, ARM_R_IN = 6;
  }

  public static final class DIGITAL {
    public static final int
      FEEDER_BREAK_BEAM = 0, 
      FEEDER_BOTTOM_BREAK_BEAM = 1;
  }

  public static final class TUNING {
    public static final boolean 
      CLIMBER = true,
      COLORSENSOR = false,
      DRIVETRAIN = false,
      FEEDER = false,
      SHOOTERS = false,
      TURRET = false,
      VISION = true;
  }

  public static final class AGITATOR {
    public static final double
      OPEN_LOOP_EXHAUST = 0.90,
      OPEN_LOOP_LOAD = 0.45,
      OPEN_LOOP_SHOOT = 0.45,
      OPEN_LOOP_RAMP = 0.125;  // TODO tune
  }
  
  public static final class CLIMBER {
    public static final double
      OPEN_LOOP_ZERO = 0.1,
      DEADBAND = 0.2,
      TIMEOUT_ZERO = 5.0,
      GEAR_RATIO = 72.0 / 14.0,
      TICKS_TO_TOP = 10000.0,  // TODO
      TICKS_PER_UNIT_DISTANCE = 1.0 / TICKS_TO_TOP,
      LIMIT_F = TICKS_TO_TOP,
      POSITION_DEADBAND = 0.02,  // Tune
      P = 0.0,
      I = 0.0,
      D = 0.0,
      F = 0.0;
  }

  public static final class COLORSENSOR {    
    public static final I2C.Port I2C_PORT = I2C.Port.kMXP;

    public static final int DISTANCE_MIN = 120;
    public static final Color 
      MATCH_BLUE = new Color(0.214, 0.325, 0.46),
      MATCH_RED = new Color(0.479, 0.378, 0.114);
  }

  public static final class DIAGNOSTICS {
    public static final double RUMBLE_PERCENT = 0.2;
    public static final DiagnosticState
      FAULT_CARGO = new DiagnosticState(new LEDColor(255, 0, 255)),
      FAULT_TURRET = new DiagnosticState(new LEDColor(255, 255, 0), true),
      CLIMBING = new DiagnosticState(new LEDColor(0, 255, 255)),
      CARGO_LOADED = new DiagnosticState(new LEDColor(76, 0, 76)),
      CAN_PRESS_SHOOT = new DiagnosticState(new LEDColor(0, 255, 0));
  }

  public static final class DRIVETRAIN {
    public static final double
      TIMEOUT_DISABLED_COAST = 5.0,
      THROTTLE_SLEW_LIMIT = 1.20,  // % output per second
      THROTTLE_DEADBAND = 0.04,
      TURNING_DEADBAND = 0.035;

    public static final int CURRENT_LIMIT = 30;

    public static final double 
      WHEEL_DIAMETER = 6.0,
      WHEEL_TRACK_WIDTH_INCHES = 26.0,
      WHEEL_SCRUB_FACTOR = 1.02,
      GEAR_RATIO = 10.71,
      VELOCITY_MAX = 5290.0,

      FEED_FORWARD_GAIN_STATIC = 0.31492,  // Tuned 3/6
      FEED_FORWARD_GAIN_VELOCITY = 2.87,
      FEED_FORWARD_GAIN_ACCEL = 0.81851,
      VOLTAGE_P = 0.001,  // Tuned 3/6
      VOLTAGE_I = 0.0,
      VOLTAGE_D = 0.0,
      VELOCITY_P = 0.0005,
      VELOCITY_I = 0.0,
      VELOCITY_D = 0.0,
      VELOCITY_F = 1.0 / VELOCITY_MAX;
      //Left and right should be tuned via a step response to a velocity change - P = V, D = A, I = dist

    public static final boolean IS_LEFT_ENCODER_INVERTED = false;
    public static final boolean IS_RIGHT_ENCODER_INVERTED = true;

    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(WHEEL_TRACK_WIDTH_INCHES * 0.0254);
    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
      FEED_FORWARD_GAIN_STATIC, FEED_FORWARD_GAIN_VELOCITY, FEED_FORWARD_GAIN_ACCEL
    );

    public static final double
      MAX_AUTO_VOLTS = 10.0,
      MAX_METERS_PER_SECOND = 3.6,
      MAX_METERS_PER_SECOND_SQUARED = 1.5,
      MAX_METERS_PER_SECOND_SLOW = 2.0,
      MAX_METERS_PER_SECOND_SQUARED_SLOW = 1.0,
      RAMSETE_B = 1.0,     // FRC recommends 2.0
      RAMSETE_ZETA = 0.5;  // FRC recommends 0.7
      
    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(FEED_FORWARD, KINEMATICS, MAX_AUTO_VOLTS);
    public static final TrajectoryConfig 
      PATH_CONFIG_F = new TrajectoryConfig(MAX_METERS_PER_SECOND, MAX_METERS_PER_SECOND_SQUARED).setKinematics(KINEMATICS).addConstraint(VOLTAGE_CONSTRAINT),
      PATH_CONFIG_R = new TrajectoryConfig(MAX_METERS_PER_SECOND, MAX_METERS_PER_SECOND_SQUARED).setKinematics(KINEMATICS).addConstraint(VOLTAGE_CONSTRAINT).setReversed(true),
      PATH_CONFIG_F_SLOW = new TrajectoryConfig(MAX_METERS_PER_SECOND_SLOW, MAX_METERS_PER_SECOND_SQUARED_SLOW).setKinematics(KINEMATICS).addConstraint(VOLTAGE_CONSTRAINT),
      PATH_CONFIG_R_SLOW = new TrajectoryConfig(MAX_METERS_PER_SECOND_SLOW, MAX_METERS_PER_SECOND_SQUARED_SLOW).setKinematics(KINEMATICS).addConstraint(VOLTAGE_CONSTRAINT).setReversed(true);
  }

  public static final class FEEDER {    
    public static final double
      OPEN_LOOP_EXHAUST = 0.2,
      OPEN_LOOP_LOAD = 0.3,
      OPEN_LOOP_SHOOT = 0.5,
      OPEN_LOOP_RAMP = 0.25,  // TODO tune
      TIMEOUT_EXHAUST = 1.0;
  }

  public static final class INFRASTRUCTURE {
    public static boolean CAMERA_STREAM = false;
  }

  public static final class INTAKE {
    public static final double
      OPEN_LOOP = 0.32846,
      OPEN_LOOP_RAMP = 0.4,
      TIMEOUT_STOW = 1.25;
  }

  public static final class SHOOTER {
    public static int STABLE_COUNTS = 2;
    public static final double
      OPEN_LOOP_REV_SECONDS = 1.0,
      RPM_MAX = 6380.0 * 1.105,  //  Tuned 3/15
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
    public static int STABLE_COUNTS = SHOOTER.STABLE_COUNTS;
    public static final double
      RPM_MAX = 6380.0 * 1.25,  //  Tuned 3/15
      RPM_ERROR_ALLOWED = SHOOTER.RPM_ERROR_ALLOWED * 2.0,  // Tuned 3/1, 25-50 seem to work well TODO try 30 again
      RPM_TRIM = SHOOTER.RPM_TRIM * 2.0,
      TICKS_PER_REV = 2048.0,
      P = 0.02,  // Probably between 0.0075 and 0.25
      F =  TICKS_PER_REV / RPM_MAX / 60.0 * 10.0,
      WHEEL_DIAMETER = 2.0,
      GEAR_RATIO = 36.0 / 55.0;
  }

  public static final class TURRET {
    public static int STABLE_COUNTS = 4;
    public static final double
      STICK_GAIN = 150.0,
      STICK_DEADBAND = 0.1,
      GEAR_RATIO = 72.0 / 14.0 * 154.0 / 16.0,
      VELOCITY_MAX = 21500.0;

    public static final ServoMotorSubsystemConstants SERVO = new ServoMotorSubsystemConstants();
    static {
      SERVO.kMasterConstants.id = CAN.TURRET;
      SERVO.kMasterConstants.invert_motor = false;
      SERVO.kMasterConstants.invert_sensor_phase = false;
      SERVO.kSupplyContinuousCurrentLimit = 20;
      SERVO.kSupplyPeakCurrentLimit = 40;
      SERVO.kSupplyPeakCurrentDuration = 10; // ms

      SERVO.kMinUnitsLimit = -210.0;
      SERVO.kMaxUnitsLimit = 113.0;
      SERVO.kHomePosition = -42.5;
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
    public static int STABLE_COUNTS = 3;
    public static final double
      CAMERA_MOUNTING_HEIGHT = 44.5,  // Inches
      CAMERA_MOUNTING_ANGLE = 27.6, // Degrees, tuned 2/27
      CAMERA_MOUNTING_OFFSET = 0.0;  // Tuned 3/4

    public static ShootMap MAP = new ShootMap();
    static {
      MAP.add(75.0, 1450.0, 2800.0);
      MAP.add(90.0, 1500.0, 3000.0);
      MAP.add(120.0, 1600.0, 3200.0);
      MAP.add(150.0, 1700.0, 3400.0);
      MAP.add(170.0, 1850.0, 3700.0);
    }
  }

  public static final class FIELD {
    public static final double VISION_TAPE_INCHES = 102.0;
  }
}
