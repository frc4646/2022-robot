package frc.robot;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.robot.util.DiagnosticState;
import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;

public final class Constants {
  public static int CAN_TIMEOUT = 100;

  public static final class CAN {
    public static final int
      PNEUMATIC_CONTROL_MODULE = 0, POWER_DISTRIBUTION_PANEL = 2,
      DRIVETRAIN_FL = 21, DRIVETRAIN_BL = 22, DRIVETRAIN_BR = 23, DRIVETRAIN_FR = 24,
      SHOOTER_L = 6, SHOOTER_R = 7, TALON_SHOOTER_L = 25, TALON_SHOOTER_R = 27,
      CLIMBER_L = 13, CLIMBER_R = 14,
      INTAKE = 5, FEEDER = 8, TURRET = 26, HOOD = 11, AGITATOR = 12, CANIFIER = 9, CANDLE = 15;
  }

  public static final class Solenoid {
    public static final int
      INTAKE_L_OUT = 0, INTAKE_L_IN = 1, INTAKE_R_OUT = 2, INTAKE_R_IN = 3,
      CLIMBER_L_OUT = 4, CLIMBER_L_IN = 5, CLIMBER_R_OUT = 6, CLIMBER_R_IN = 7;
  }

  public static final class Digital {
    public static final int FEEDER_BREAK_BEAM = 0;
  }

  public static final class Agitator {
    public static final double OPEN_LOOP_RAMP = 0.25;  // TODO tune
  }

  public static final class Diagnostic {
    public static final int LED_COUNT = 58;
    public static final CANdleConfiguration LED_CONFIG = new CANdleConfiguration();
    static {
      LED_CONFIG.statusLedOffWhenActive = true;
      LED_CONFIG.disableWhenLOS = false;
      LED_CONFIG.stripType = LEDStripType.GRB;
      LED_CONFIG.brightnessScalar = 1.0;
      LED_CONFIG.vBatOutputMode = VBatOutputMode.Off;
    }

    public static final double RUMBLE_PERCENT = 0.2;  // TODO tune
    public static final DiagnosticState
      FAULT_CARGO = new DiagnosticState(Diagnostics.toColor(255, 0, 255, 0.1)),
      FAULT_TURRET = new DiagnosticState(Diagnostics.toColor(255, 255, 0, 1.0), true),
      AIMING = new DiagnosticState(Diagnostics.toColor(0, 255, 255, .1)),
      CLIMBING = new DiagnosticState(Diagnostics.toColor(0, 255, 255, 0.5)),
      SHOOTING = new DiagnosticState(Diagnostics.toColor(255, 0, 0, .1));
  }

  public static final class Drivetrain {
    public static final boolean TUNING = false;

    public static final double 
      WHEEL_TRACK_WIDTH_INCHES = 26.0,
      WHEEL_SCRUB_FACTOR = 1.02,
      WHEEL_DIAMETER = 6.0;

    public static final double
      THROTTLE_SLEW_LIMIT = 1.0,  // % output per second
      THROTTLE_DEADBAND = 0.04,
      TURNING_DEADBAND = 0.035;
        
    public static final double VOLTAGE_COMPENSATION = 9.0;  // TODO INCREASE BEFORE COMPETITION
    public static final int CURRENT_LIMIT = 30;
    //public static final int kDriveCurrentUnThrottledLimit = 80; // TODO use case?

    public static final double
      FEED_FORWARD_GAIN_STATIC = 0.0,  // TODO
      FEED_FORWARD_GAIN_VELOCITY = 0.0,  // TODO
      FEED_FORWARD_GAIN_ACCEL = 0.0,  // TODO
      P = 0.0,
      I = 0.0,
      D = 0.0,
      F = 0.0;  // TODO
  }

  public static final class Feeder {
    public static final double
      OPEN_LOOP = 0.5,
      OPEN_LOOP_RAMP = 0.25;  // TODO tune
  }

  public static final class Hood {
    public static final boolean TUNING = false;

    public static final double DEGREES_DEFAULT = 65.0;  // TODO tune
  }

  public static final class Intake {
    public static final double
      OPEN_LOOP = 1.0,
      OPEN_LOOP_RAMP = 0.25;  // TODO tune
  }

  public static final class Shooter {
    public static final boolean TUNING = false;

    public static int  RPM_STABLE_COUNTS = 5;
    public static final double
      OPEN_LOOP = .5,
      OPEN_LOOP_REV_SECONDS = 1.5,
      RPM_MAX = 6380.0 * .94,
      RPM_DEFAULT = RPM_MAX / 2.0,
      RPM_ERROR_ALLOWED = 250.0,
      TICKS_PER_REV = 2048.0,
      P = 0.0,  // Probably between 0.0075 and 0.25
      I = 0.0,  // Use 0 if possible. But if we do use non-zero, make sure to use i zone
      D = 0.0,  // Stay 0
      F =  TICKS_PER_REV / RPM_MAX / 60.0 * 10.0;  // Equals 0.05029
  }

  public static final class Turret {
    public static final boolean TUNING = true;

    public static final double
      OPEN_LOOP = 0.3,
      OPEN_LOOP_DEADBAND = 0.8,
      OPEN_LOOP_GAIN = 4.0,
      GEAR_RATIO = 24.0 / 8.0 * 240.0 / 14.0,  // Number > 1 means "geared down" Real: 14:72 & 16:154
      VELOCITY_MAX = 2200.0;  // TODO real value

    public static final ServoMotorSubsystemConstants SERVO = new ServoMotorSubsystemConstants();
    static {
      SERVO.kMasterConstants.id = CAN.TURRET;
      SERVO.kMasterConstants.invert_motor = false;
      SERVO.kMasterConstants.invert_sensor_phase = false;

      SERVO.kHomePosition = 180.0;  // TODO real value
      SERVO.kTicksPerUnitDistance = 2048.0 * GEAR_RATIO / 360.0;

      SERVO.kKp = 0.0;
      SERVO.kKd = 0.0;
      SERVO.kKf = 1023.0 * VELOCITY_MAX;
      SERVO.kKa = 0.0;
      SERVO.kCruiseVelocity = 20000; // Ticks / 100ms  TODO VELOCITY_MAX * 0.975;  // .95-.975 might work well
			SERVO.kAcceleration = 40000; // Ticks / 100ms / s TODO MOTION_MAGIC_VELOCITY * 3.0;  // 3 or 6 might work well
      SERVO.kDeadband = 0.0;

      SERVO.kPositionKp = 0.015;
      SERVO.kPositionKd = 0.0;
      SERVO.kPositionKf = 0.0;
      SERVO.kPositionDeadband = 0.1 * SERVO.kTicksPerUnitDistance; // Ticks

      SERVO.kMinUnitsLimit = 80.0;  // TODO real value
      SERVO.kMaxUnitsLimit = 280.0;  // TODO real value

			// SERVO.kContinuousCurrentLimit = 20; // amps
			// SERVO.kPeakCurrentLimit = 40; // amps
			// SERVO.kPeakCurrentDuration = 10; // ms

      // TODO SERVO.kRecoverPositionOnReset = true;
    }
  }

  public static final class Vision {
    public static final double
      HORIZONTAL_FOV = 54.0,  // Degrees (LL1: 54.0, LL2: 59.6)
      VERTICAL_FOV = 41.0,  // Degrees (LL1: 41.0, LL2: 49.7)
      CAMERA_MOUNTING_HEIGHT = 40.0,  // Inches
      CAMERA_MOUNTING_ANGLE = 21.75;  // Degrees

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
      RPM_MAP = new InterpolatingTreeMap<>(),
      ANGLE_MAP = new InterpolatingTreeMap<>();
    static {
      RPM_MAP.put(new InterpolatingDouble(114.0), new InterpolatingDouble(2320.0));
      RPM_MAP.put(new InterpolatingDouble(127.0), new InterpolatingDouble(2495.0));
      RPM_MAP.put(new InterpolatingDouble(147.0), new InterpolatingDouble(2750.0));
      RPM_MAP.put(new InterpolatingDouble(163.0), new InterpolatingDouble(2910.0));
      RPM_MAP.put(new InterpolatingDouble(196.0), new InterpolatingDouble(3200.0));
    }
    static {
      ANGLE_MAP.put(new InterpolatingDouble(114.0), new InterpolatingDouble(2320.0));
      ANGLE_MAP.put(new InterpolatingDouble(127.0), new InterpolatingDouble(2495.0));
      ANGLE_MAP.put(new InterpolatingDouble(147.0), new InterpolatingDouble(2750.0));
      ANGLE_MAP.put(new InterpolatingDouble(163.0), new InterpolatingDouble(2910.0));
      ANGLE_MAP.put(new InterpolatingDouble(196.0), new InterpolatingDouble(3200.0));
    }
  }

  public static final class Field {
    public static final double VISION_TAPE_INCHES = 102.0;
  }
}
