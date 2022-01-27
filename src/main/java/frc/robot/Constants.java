package frc.robot;

import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;

public final class Constants {
  public static final class Ports {
    public static final int
      POWER_DISTRIBUTION_PANEL = 0,  DRIVETRAIN_FL = 1, DRIVETRAIN_BR = 2, DRIVETRAIN_BL = 3, DRIVETRAIN_FR = 4,
      INTAKE = 5, SHOOTER_L = 6, SHOOTER_R = 7, INDEXER = 8, GYRO = 9;
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
  }

  public static final class Shooter {
    public static final double REV_TIME = 2.0;  // Seconds
    public static final double SPEED_NO_TARGETS = 0.0;  // TODO
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> RPM_MAP = new InterpolatingTreeMap<>();
    static {
      RPM_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(4500.0));
    }
  }

  public static final class Vision {
    public static final double HORIZONTAL_FOV = 54.0;  // Degrees (LL1: 54.0, LL2: 59.6)
    public static final double VERTICAL_FOV = 41.0;  // Degrees (LL1: 41.0, LL2: 49.7)
    public static final double CAMERA_MOUNTING_HEIGHT = 24.0;  // Inches TODO real value
    public static final double CAMERA_MOUNTING_ANGLE = 24.0;  // Degrees TODO real value
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
