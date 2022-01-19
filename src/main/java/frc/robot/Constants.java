package frc.robot;

public final class Constants {
  public static final class Ports {
    public static final int DRIVETRAIN_FL = 0;
    public static final int DRIVETRAIN_FR = 1;
    public static final int DRIVETRAIN_BL = 2;
    public static final int DRIVETRAIN_BR = 3;
    public static final int INTAKE = 0;
  }

  public static final class Drivetrain {
    public static final double WHEEL_TRACK_WIDTH_INCHES = 28.0; // TODO actual value
    public static final double WHEEL_SCRUB_FACTOR = 1.02;

    public static final double THROTTLE_DEADBAND = 0.04;
    public static final double TURNING_DEADBAND = 0.035;
        
    public static final int CURRENT_LIMIT = 30;
    //public static final int kDriveCurrentUnThrottledLimit = 80; // TODO use case?
  }
}
