package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.util.InterpolatingDouble;

public class Vision extends SmartSubsystem {
  public enum LEDMode { PIPELINE, OFF, BLINK, ON }         // Order must match Limelight docs
  public enum CamMode { VISION_PROCESSOR, DRIVER_CAMERA }  // Order must match Limelight docs
  public static class DataCache {
    public double xDegrees;     // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees;     // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public double area;         // Target Area (0% of image to 100% of image)
    public boolean seesTarget;  // Whether the limelight has any valid targets (0 or 1)
    public int modeLED = LEDMode.PIPELINE.ordinal();
    public double distance;
    public boolean inShootRange = false;
    public double filteredDistance, filteredArea;
  }

  private final double RPM_MAP_KEY_INVALID = -1.0;
  private final double HEIGHT_VISION_TAPE_TO_CAMERA = Constants.FIELD.VISION_TAPE_INCHES - Constants.VISION.CAMERA_MOUNTING_HEIGHT;

  private final NetworkTable table;
  private final LinearFilter filterDistance = LinearFilter.movingAverage(5), filterArea = LinearFilter.movingAverage(5);
  private final DataCache cache = new DataCache();
  private int stableCounts = 0;  // TODO use moving average of linear filter on vision data instead?

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setLED(LEDMode.OFF);
  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  @Override
  public void cacheSensors() {
    cache.modeLED = (int) table.getEntry("ledMode").getDouble(1.0);
    cache.xDegrees = table.getEntry("tx").getDouble(0.0);
    cache.yDegrees = table.getEntry("ty").getDouble(0.0);
    cache.area = table.getEntry("ta").getDouble(0.0);
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    cache.distance = getGroundDistanceToHubInches();
    cache.inShootRange = cache.distance > Constants.VISION.DISTANCE_USABLE_MIN && cache.distance < Constants.VISION.DISTANCE_USABLE_MAX;

    if (cache.seesTarget) {
      cache.filteredDistance = filterDistance.calculate(cache.distance);
      cache.filteredArea = filterArea.calculate(cache.area);
    } else {
      filterDistance.reset();
      filterArea.reset();
      cache.filteredDistance = 0.0;
      cache.filteredArea = 0.0;
    }
    if (!cache.inShootRange) {
      cache.xDegrees = 0.0;
      cache.yDegrees = 0.0;
      cache.area = 0.0;
      cache.seesTarget = false;
      cache.distance = 0.0;
    }
    stableCounts++;
    if (!isTargetPresent()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Vision: Target", cache.seesTarget);
    SmartDashboard.putNumber("Vision: Distance", cache.distance);
    SmartDashboard.putNumber("Vision: X", cache.xDegrees);
    if (Constants.VISION.TUNING) {
      SmartDashboard.putNumber("Vision: Y", cache.yDegrees);
      SmartDashboard.putNumber("Vision: Area", cache.area);
      SmartDashboard.putNumber("Vision: Distance Filtered", cache.filteredDistance);
      SmartDashboard.putNumber("Vision: Area Filtered", cache.filteredArea);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setLED(LEDMode.ON);
  }

  @Override
  public void onDisable() {
    setLED(LEDMode.OFF);
  }

  public void setLED(LEDMode mode) {
    int ledMode = mode.ordinal();
    if (ledMode != cache.modeLED) {
      table.getEntry("ledMode").setNumber(ledMode);
      cache.modeLED = ledMode;
    }
  }

  /** See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#. */
  public double getGroundDistanceToHubInches() {
    if (isTargetPresent()) {
      return HEIGHT_VISION_TAPE_TO_CAMERA / Math.tan(Math.toRadians(cache.yDegrees + Constants.VISION.CAMERA_MOUNTING_ANGLE)) - Constants.VISION.CAMERA_MOUNTING_OFFSET;
    }
    return RPM_MAP_KEY_INVALID;
  }

  public double getHoodDegrees() {
    if (isTargetPresent()) {
      return Constants.VISION.ANGLE_MAP.getInterpolated(new InterpolatingDouble(getGroundDistanceToHubInches())).value;
    }
    return Constants.HOOD.DEGREES_DEFAULT;
  }

  public double getShooterRPM() {
    if (isTargetPresent()) {
      return Constants.VISION.RPM_MAP.getInterpolated(new InterpolatingDouble(getGroundDistanceToHubInches())).value;
    }
    return Constants.SHOOTER.RPM_DEFAULT;
  }

  public double getDegreesX() {
    return cache.xDegrees;
  }

  public boolean isTargetPresent() { return cache.seesTarget; }  
  public boolean isStable() { return stableCounts >= Constants.VISION.STABLE_COUNTS; }
  public boolean isInShootRange() { return cache.inShootRange; };

  @Override
  public void runTests() {
    Test.add(this, "Target", cache.seesTarget);
  }
}
