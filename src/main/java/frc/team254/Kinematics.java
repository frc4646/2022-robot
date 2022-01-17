package frc.team254;

import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;
import frc.team254.util.DriveSignal;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */
public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
     */
    public static DriveSignal inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveSignal(velocity.dx, velocity.dx);
        }
        double delta_v = Constants.Drive.WHEEL_TRACK_WIDTH_INCHES * velocity.dtheta / (2 * Constants.Drive.WHEEL_SCRUB_FACTOR);
        return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}
