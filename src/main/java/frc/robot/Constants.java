package frc.robot;

public final class Constants {
    public static final class Ports {
        public static final int DRIVE_FL = 0;
        public static final int DRIVE_FR = 0;
        public static final int DRIVE_BL = 0;
        public static final int DRIVE_BR = 0;
    }

    public static final class Drive {
        public static final double WHEEL_TRACK_WIDTH_INCHES = 28.0; // TODO actual value
        public static final double WHEEL_SCRUB_FACTOR = 1.02;

        public static final double THROTTLE_DEADBAND = 0.04;
        public static final double TURNING_DEADBAND = 0.035;
        
        public static final int CURRENT_LIMIT = 30;
        //public static final int kDriveCurrentUnThrottledLimit = 80; // TODO use case?
    }
}
