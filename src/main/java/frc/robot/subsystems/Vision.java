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
    public double areaRaw;         // Target Area (0% of image to 100% of image)
    public boolean seesTarget;  // Whether the limelight has any valid targets (0 or 1)
    public int modeLED = LEDMode.PIPELINE.ordinal();
    public double distanceRaw;
    public boolean inShootRange = false;
    public double distanceFiltered, areaFiltered;
  }

  private final double RPM_MAP_KEY_INVALID = -1.0;
  private final double HEIGHT_VISION_TAPE_TO_CAMERA = Constants.FIELD.VISION_TAPE_INCHES - Constants.VISION.CAMERA_MOUNTING_HEIGHT;
  private final double CUTOFF_FREQUENCY = 100.0;
  private final double FILTER_TIME_CONSTANT = 1.0 / (2.0 * Math.PI * CUTOFF_FREQUENCY);

  private final NetworkTable table;
  private final LinearFilter filterDistance = LinearFilter.singlePoleIIR(FILTER_TIME_CONSTANT, .02), filterArea = LinearFilter.singlePoleIIR(FILTER_TIME_CONSTANT, .02);
  private final DataCache cache = new DataCache();
  private int seesTargetCounts = 0, noTargetCounts = 0;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setLED(LEDMode.OFF);
  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  @Override
  public void cacheSensors() {
    // Raw Data
    cache.modeLED = (int) table.getEntry("ledMode").getDouble(1.0);
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    cache.xDegrees = table.getEntry("tx").getDouble(0.0);
    cache.yDegrees = table.getEntry("ty").getDouble(0.0);
    cache.areaRaw = table.getEntry("ta").getDouble(0.0);
    // Calculated Data
    seesTargetCounts++;
    noTargetCounts++;
    if (!isTargetPresent()) {
      seesTargetCounts = 0;
    } else {
      noTargetCounts = 0;
    }
    cache.distanceRaw = getGroundDistanceToHubInches();
    if (cache.seesTarget) {
      cache.areaFiltered = filterArea.calculate(cache.areaRaw);
      cache.distanceFiltered = filterDistance.calculate(cache.distanceRaw);
    }
    // TODO reset filtered values if no targets counts reaches threshold
    cache.inShootRange = cache.distanceRaw > Constants.VISION.DISTANCE_USABLE_MIN && cache.distanceRaw < Constants.VISION.DISTANCE_USABLE_MAX;
    if (!cache.inShootRange) {
      cache.seesTarget = false;
      cache.xDegrees = 0.0;
      cache.yDegrees = 0.0;
      cache.areaRaw = 0.0;
      cache.distanceRaw = 0.0;
    }
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Vision: Target", cache.seesTarget);
    SmartDashboard.putNumber("Vision: Distance", cache.distanceRaw);
    SmartDashboard.putNumber("Vision: X", cache.xDegrees);
    if (Constants.VISION.TUNING) {
      SmartDashboard.putNumber("Vision: Y", cache.yDegrees);
      SmartDashboard.putNumber("Vision: Area", cache.areaRaw);
      SmartDashboard.putNumber("Vision: Distance Filtered", cache.distanceFiltered);
      SmartDashboard.putNumber("Vision: Area Filtered", cache.areaFiltered);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    filterDistance.reset();
    filterArea.reset();
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
      return Constants.VISION.ANGLE_MAP.getInterpolated(new InterpolatingDouble(cache.distanceFiltered)).value;
    }
    return Constants.HOOD.DEGREES_DEFAULT;
  }

  public double getShooterRPM() {
    if (isTargetPresent()) {
      return Constants.VISION.RPM_MAP.getInterpolated(new InterpolatingDouble(cache.distanceFiltered)).value;
    }
    return Constants.SHOOTER.RPM_DEFAULT;
  }

  public double getDegreesX() {
    return cache.xDegrees;
  }

  public boolean isTargetPresent() { return cache.seesTarget; }
  public boolean isStable() { return seesTargetCounts >= Constants.VISION.STABLE_COUNTS; }
  public boolean isInShootRange() { return cache.inShootRange; };

  @Override
  public void runTests() {
    Test.add(this, "Target", cache.seesTarget);
  }
}
