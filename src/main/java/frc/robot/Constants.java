package frc.robot;

import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.robot.util.DiagnosticState;
import frc.robot.util.ShootMap;
import frc.team4646.LEDColor;
import frc.team4646.PID;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
      CLIMBER_L = 14, CLIMBER_R = 13,
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
      BREAK_BEAM_SHOOTER = 0,
      BREAK_BEAM_HOPPER = 1,
      BREAK_BEAM_INDEXER = 2;
  }

  public static final class TUNING {
    public static final boolean 
      CLIMBER = false,
      COLORSENSOR = false,
      DRIVETRAIN = false,
      FEEDER = false,
      ROBOT_STATE = false,
      SHOOTERS = false,
      TURRET = false,
      VISION = false;
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
      TICKS_TO_TOP = 227300.0 * 3.0,
      TICKS_PER_UNIT_DISTANCE = TICKS_TO_TOP,
      SOFT_LIMIT_R = TICKS_TO_TOP * 0.0375,
      SOFT_LIMIT_F = TICKS_TO_TOP * 1.00,
      POSITION_DEADBAND = 0.02;  // TODO tune
    public static PID PID = new PID(0.32, 0.0, 0.0);
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
      FAULT_CLIMBER = new DiagnosticState(new LEDColor(255, 0, 255)),
      FAULT_TURRET = new DiagnosticState(new LEDColor(255, 165, 0), true),
      CLIMBING = new DiagnosticState(new LEDColor(255, 105, 180)),
      CARGO_LOADED = new DiagnosticState(new LEDColor(76, 0, 76)),
      CAN_PRESS_SHOOT = new DiagnosticState(new LEDColor(0, 255, 0));
  }

  public static final class DRIVETRAIN {
    public static final double
      TIMEOUT_DISABLED_COAST = 5.0,
      THROTTLE_SLEW_LIMIT = 1.20,  // % output per second
      THROTTLE_DEADBAND = 0.08,
      TURNING_DEADBAND = 0.07;

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
      CRACKPOINT = 0.012;  // Tuned 3/21

    public static PID
      PID_VOLTAGE = new PID(0.001, 0.0, 0.0),  // Tuned 3/6
      PID_VELOCITY = new PID(0.0005, 0.0, 0.0, 1.0 / VELOCITY_MAX);  // with crackpoint: F=0.00017, P=0.0001???

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
      RAMSETE_B = 1.0,     // FRC recommends 2.0, TODO probably can increase
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
      OPEN_LOOP_EXHAUST = 0.5,
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
      OPEN_LOOP = 0.50,
      OPEN_LOOP_RAMP = 0.4,
      TIMEOUT_STOW = 1.25;
  }

  public static final class SHOOTER {
    public static int STABLE_COUNTS = 2;
    public static final double
      OPEN_LOOP_REV_SECONDS = 1.0,
      RPM_MAX = 6380.0,  //  Tuned 3/15
      RPM_ERROR_ALLOWED = 70.0,  // TODO TURN THIS DOWN
      RPM_TRIM = 150.0,
      DEADBAND = 0.05,
      TICKS_PER_REV = 2048.0,
      CRACKPOINT = 0.03968;
    public static PID PID = new PID(0.05, 0.0, 0.1, 0.04535);  // TICKS_PER_REV / RPM_MAX / 60.0 * 10.0,
  }

  public static final class SHOOTER_TOP {
    public static int STABLE_COUNTS = SHOOTER.STABLE_COUNTS;
    public static final double
      RPM_MAX = 6380.0,  //  Tuned 3/15
      RPM_ERROR_ALLOWED = SHOOTER.RPM_ERROR_ALLOWED * 1.5,  // TODO TURN THIS DOWN
      RPM_TRIM = SHOOTER.RPM_TRIM * 2.0,
      TICKS_PER_REV = 2048.0,
      CRACKPOINT = 0.03968;
    public static PID PID = new PID(0.02, 0.0, 0.08, 0.047);  // TICKS_PER_REV / RPM_MAX / 60.0 * 10.0,
  }

  public static final class TURRET {
    public static int STABLE_COUNTS = 4;
    public static final double
      STICK_GAIN = 150.0,
      STICK_DEADBAND = 0.1,
      ERROR_ALLOWED_DEGREES = 1.0,  // TODO TUNE THIS
      GEAR_RATIO = 72.0 / 14.0 * 154.0 / 16.0,
      VELOCITY_MAX = 21500.0,
      GAIN_STABILITY = 1.0;

    public static final ServoMotorSubsystemConstants SERVO = new ServoMotorSubsystemConstants();
    static {
      SERVO.kMasterConstants.id = CAN.TURRET;
      SERVO.kMasterConstants.invert_motor = false;
      SERVO.kMasterConstants.invert_sensor_phase = false;
      SERVO.kSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(false, 20.0, 40.0, 10.0);  // TODO

      SERVO.kMinUnitsLimit = -210.0;
      SERVO.kMaxUnitsLimit = 113.0;
      SERVO.kHomePosition = -42.5;
      SERVO.kTicksPerUnitDistance = SERVO.kMasterConstants.ticksPerMotorRotation * GEAR_RATIO / 360.0;

      SERVO.kPositionPID = new PID(0.02, 0.0008, 0.25);
      SERVO.kPositionIZone = 40.0;
      SERVO.kPositionMaxIntegralAccumulator = 20000.0;
      SERVO.kPositionDeadband = 0.1 * SERVO.kTicksPerUnitDistance; // Ticks

      SERVO.kMotionMagicPID = new PID(0.6, 0.0, 1.2, 1023.0 / VELOCITY_MAX);
      SERVO.kCruiseVelocity = VELOCITY_MAX * 0.95;
      SERVO.kAcceleration = SERVO.kCruiseVelocity * 1.0;
      SERVO.kMotionMagicDeadband = 1.0 * SERVO.kTicksPerUnitDistance; // Ticks  // TODO REDUCE
    }
  }

  public static final class VISION {
    public static int STABLE_COUNTS = 3;
    public static final double
      CAMERA_MOUNTING_HEIGHT = 44.5,  // Inches
      CAMERA_MOUNTING_ANGLE = 29.0, // 30.4, // Degrees, tuned 2/27
      CAMERA_MOUNTING_OFFSET = 0.0;  // Tuned 3/4

    public static ShootMap MAP = new ShootMap();
    static {
      MAP.add(75.0, 1200.0, 2800.0);
      MAP.add(95.0, 1250.0, 2900.0);
      MAP.add(110.0, 1420.0, 3020.0);
      MAP.add(125.0, 1500.0, 3100.0);
      MAP.add(150.0, 1625.0, 3500.0);
      MAP.add(170.0, 1750.0, 3700.0);
      MAP.add(194.0, 1850.0, 3900.0);
    }
  }

  public static final class FIELD {
    public static final double VISION_TAPE_INCHES = 102.0;
  }
}
