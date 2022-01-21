package frc.robot;

public final class Constants {
  public static final class Ports {
    public static final int POWER_DISTRIBUTION_PANEL = 0;
    public static final int DRIVETRAIN_FL = 4;
    public static final int DRIVETRAIN_FR = 1;
    public static final int DRIVETRAIN_BL = 2;
    public static final int DRIVETRAIN_BR = 3;
    public static final int INTAKE = 5;
  }

  public static final class Drivetrain {
    public static final double WHEEL_TRACK_WIDTH_INCHES = 26.0;
    public static final double WHEEL_SCRUB_FACTOR = 1.02;

    public static final double THROTTLE_DEADBAND = 0.04;
    public static final double TURNING_DEADBAND = 0.035;
        
    public static final int CURRENT_LIMIT = 30;
    //public static final int kDriveCurrentUnThrottledLimit = 80; // TODO use case?
  }

  public static final class Vision {
    public static final double HORIZONTAL_FOV = 59.6;  // Degrees TODO makes sure we have limelight2
    public static final double VERTICAL_FOV = 49.7;  // Degrees TODO makes sure we have limelight2
    public static final double CAMERA_MOUNTING_HEIGHT = 24.0;  // Inches TODO real value
    public static final double CAMERA_MOUNTING_ANGLE = 24.0;  // TODO real value
  }

  public static final class Field {
    public static final double HUB_HEIGHT = 104.0; // Inches
    public static final double HUB_DIAMETER = 53.375; // Inches
    public static final double VISION_TAPE_BOTTOM = 101.625; // Inches
    public static final double VISION_TAPE_HEIGHT = (HUB_HEIGHT + VISION_TAPE_BOTTOM) / 2.0; // Inches
    public static final double VISION_TAPE_WIDTH = 5.0; // Inches
    public static final double VISION_TAPE_GAP = 5.5; // Inches
  }
}
