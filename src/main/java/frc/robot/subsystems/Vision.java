package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  public enum LEDMode { PIPELINE, OFF, BLINK, ON }         // Order must match Limelight docs
  public enum CamMode { VISION_PROCESSOR, DRIVER_CAMERA }  // Order must match Limelight docs
  public static class DataCache {
    public double xDegrees;     // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees;     // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public double area;         // Target Area (0% of image to 100% of image)
    public boolean seesTarget;  // Whether the limelight has any valid targets (0 or 1)
  }

  private final double HEIGHT_VISION_TAPE_TO_CAMERA = Constants.Field.VISION_TAPE_HEIGHT - Constants.Vision.CAMERA_MOUNTING_HEIGHT;

  private final NetworkTable table;
  private final DataCache cache = new DataCache();

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  public void cacheSensors() {
    cache.xDegrees = table.getEntry("tx").getDouble(0.0);
    cache.yDegrees = table.getEntry("ty").getDouble(0.0);
    cache.area = table.getEntry("ta").getDouble(0.0);
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Limelight X: ", cache.xDegrees);
    SmartDashboard.putNumber("Limelight Y: ", cache.yDegrees);
    SmartDashboard.putBoolean("Limelight Target: ", cache.seesTarget);
  }

  public void setLED(LEDMode mode) {
    int ledMode = mode.ordinal();
    table.getEntry("ledMode").setNumber(ledMode);
  }

  /** See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#. */
  public double getGroundDistanceToHubInches() {
    if (isTargetPresent()) {
      return HEIGHT_VISION_TAPE_TO_CAMERA / Math.tan(Math.toRadians(cache.yDegrees) + Constants.Vision.CAMERA_MOUNTING_ANGLE);
    }
    return -1.0;
  }

  public int getShooterSpeed() {
    return 0;  // TODO ideal speed for shooter
  }

  public boolean isTargetPresent() {
    return cache.seesTarget;
  }
}
