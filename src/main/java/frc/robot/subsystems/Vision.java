package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.util.InterpolatingDouble;

public class Vision extends SmartSubsystem {
  public enum LEDMode { PIPELINE, OFF, BLINK, ON }         // Order must match Limelight docs
  public enum CamMode { VISION_PROCESSOR, DRIVER_CAMERA }  // Order must match Limelight docs
  public static class DataCache {
    public double xDegrees;     // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees;     // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public double area;         // Target Area (0% of image to 100% of image)
    public boolean seesTarget;  // Whether the limelight has any valid targets (0 or 1)
    public double distance;
  }

  private final double RPM_MAP_KEY_INVALID = -1.0;
  private final double HEIGHT_VISION_TAPE_TO_CAMERA = Constants.Field.VISION_TAPE_HEIGHT - Constants.Vision.CAMERA_MOUNTING_HEIGHT;

  private final NetworkTable table;
  private final DataCache cache = new DataCache();

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  @Override
  public void cacheSensors() {
    // TODO linear filter or median filter a goood idea? See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/index.html
    cache.xDegrees = table.getEntry("tx").getDouble(0.0);
    cache.yDegrees = table.getEntry("ty").getDouble(0.0);
    cache.area = table.getEntry("ta").getDouble(0.0);
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    cache.distance = getGroundDistanceToHubInches();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Limelight X: ", cache.xDegrees);
    SmartDashboard.putNumber("Limelight Y: ", cache.yDegrees);
    SmartDashboard.putBoolean("Limelight Target: ", cache.seesTarget);
    SmartDashboard.putNumber("Limelight Distance: ", cache.distance);
  }

  public void setLED(LEDMode mode) {
    int ledMode = mode.ordinal();
    table.getEntry("ledMode").setNumber(ledMode);
  }

  /** See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#. */
  public double getGroundDistanceToHubInches() {
    if (isTargetPresent()) {
      return HEIGHT_VISION_TAPE_TO_CAMERA / Math.tan(Math.toRadians( cache.yDegrees + Constants.Vision.CAMERA_MOUNTING_ANGLE));
    }
    return RPM_MAP_KEY_INVALID;
  }

  public double getShooterVelocityRPM() {
    if (isTargetPresent()) {
      return Constants.Shooter.RPM_MAP.getInterpolated(new InterpolatingDouble(getGroundDistanceToHubInches())).value;
    }
    return Constants.Shooter.RPM_NO_TARGETS;
  }

  public double getDegreesX() {
    return cache.xDegrees;
  }

  public boolean isTargetPresent() {
    return cache.seesTarget;
  }

  @Override
  public void runTests() {
    String sResult = (cache.seesTarget) ? "Okay" : "ERROR";
    System.out.println(String.format("Vision sees target: %s", sResult));
  }
}
