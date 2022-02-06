package frc.robot;

import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;

public final class Constants {
  public static int CAN_TIMEOUT_LONG = 100;

  public static final class CAN {
    public static final int
      PNEUMATIC_CONTROL_MODULE = 0,
      POWER_DISTRIBUTION_PANEL = 2,
      DRIVETRAIN_FL = 21, DRIVETRAIN_BL = 22, DRIVETRAIN_BR = 23, DRIVETRAIN_FR = 24,
      INTAKE = 5,
      SHOOTER_L = 6, SHOOTER_R = 7,
      FEEDER = 8,
      TURRET = 10, HOOD = 11, AGITATOR = 12;
  }
public static final class Solenoid {
    public static final int
    INTAKE_L_OUT = 0, INTAKE_L_IN = 1, INTAKE_R_OUT = 2, INTAKE_R_IN = 3,
    CLIMBER_L_OUT = 4, CLIMBER_L_IN = 5, CLIMBER_R_OUT = 6, CLIMBER_R_IN = 7;
  }

  public static final class Digital {
    public static final int FEEDER_BREAK_BEAM = 0;


  }

  public static final class Drivetrain {
    public static final double WHEEL_TRACK_WIDTH_INCHES = 26.0;
    public static final double WHEEL_SCRUB_FACTOR = 1.02;

    public static final double THROTTLE_DEADBAND = 0.04;
    public static final double TURNING_DEADBAND = 0.035;
        
    public static final int CURRENT_LIMIT = 30;
    //public static final int kDriveCurrentUnThrottledLimit = 80; // TODO use case?

    public static final double FEED_FORWARD_GAIN_STATIC = 0.0;  // TODO
    public static final double FEED_FORWARD_GAIN_VELOCITY = 0.0;  // TODO
    public static final double FEED_FORWARD_GAIN_ACCEL = 0.0;  // TODO
    public static final double P = 0.0, I = 0.0, D = 0.0, F = 0.0;  // TODO
  }

  public static final class Feeder {
    public static final double PERCENT_OPEN_LOOP = 0.5;
  }

  public static final class Intake {
    public static final double PERCENT_OPEN_LOOP = 1.0;
  }

  public static final class Shooter {
    public static final double OPEN_LOOP_PERCENT = .5;
    public static final double OPEN_LOOP_REV_SECONDS = 1.5;
    public static final double OPEN_LOOP_RPM = 2750.0;  // TODO

    public static final double ERROR_ALLOWED_RPM = 250.0;
    public static final double NO_TARGETS_RPM = 0.0;  // TODO
    public static final double P = 0.0, I = 0.0, D = 0.0, F = 0.0;  // TODO
    public static final double TICKS_PER_REV = 2048.0;
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> VOLTAGE_MAP = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> RPM_MAP = new InterpolatingTreeMap<>();
    static {
      VOLTAGE_MAP.put(new InterpolatingDouble(114.0), new InterpolatingDouble(0.42));
      VOLTAGE_MAP.put(new InterpolatingDouble(127.0), new InterpolatingDouble(0.4515));
      VOLTAGE_MAP.put(new InterpolatingDouble(147.0), new InterpolatingDouble(.5));
      VOLTAGE_MAP.put(new InterpolatingDouble(163.0), new InterpolatingDouble(.525));
      VOLTAGE_MAP.put(new InterpolatingDouble(196.0), new InterpolatingDouble(.575));
    }
    static {
      RPM_MAP.put(new InterpolatingDouble(114.0), new InterpolatingDouble(2320.0));
      RPM_MAP.put(new InterpolatingDouble(127.0), new InterpolatingDouble(2495.0));
      RPM_MAP.put(new InterpolatingDouble(147.0), new InterpolatingDouble(2750.0));
      RPM_MAP.put(new InterpolatingDouble(163.0), new InterpolatingDouble(2910.0));
      RPM_MAP.put(new InterpolatingDouble(196.0), new InterpolatingDouble(3200.0));
    }
  }

  public static final class Turret {
    public static final double ENCODER_ZERO_MIN = 0.0;  // Raw TODO actual value
    public static final double ENCODER_ZERO_MAX = 0.0;  // Raw TODO actual value
    public static final double P = 0.0, I = 0.0, D = 0.0, F = 0.0;  // TODO
  }

  public static final class Vision {
    public static final double HORIZONTAL_FOV = 54.0;  // Degrees (LL1: 54.0, LL2: 59.6)
    public static final double VERTICAL_FOV = 41.0;  // Degrees (LL1: 41.0, LL2: 49.7)
    public static final double CAMERA_MOUNTING_HEIGHT = 19.0;  // Inches
    public static final double CAMERA_MOUNTING_ANGLE = 28.0;  // Degrees
  }

  public static final class Field {
    public static final double
      HUB_HEIGHT = 104.0, // Inches
      HUB_DIAMETER = 53.375, // Inches
      VISION_TAPE_BOTTOM = 101.625, // Inches
      VISION_TAPE_HEIGHT = (HUB_HEIGHT + VISION_TAPE_BOTTOM) / 2.0, // Inches
      VISION_TAPE_WIDTH = 5.0, // Inches
      VISION_TAPE_GAP = 5.5; // Inches
  }
}
