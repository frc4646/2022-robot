package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.ShootSetpoint;
import frc.team4646.Test;

public class Vision extends SmartSubsystem {
  public enum LEDMode { PIPELINE, OFF, BLINK, ON }         // Order must match Limelight docs
  public enum CamMode { VISION_PROCESSOR, DRIVER_CAMERA }  // Order must match Limelight docs
  private class DataCache {
    public double xDegrees;     // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees;     // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public double areaRaw;         // Target Area (0% of image to 100% of image)
    public boolean seesTarget;  // Whether the limelight has any valid targets (0 or 1)
    public int modeLED = LEDMode.PIPELINE.ordinal();
    public double distanceCalculated;
    public ShootSetpoint rpmCalculated = new ShootSetpoint();
    public boolean inShootRange = false;
    public double distanceFiltered;
  }
  private class OutputCache {
    public int modeLED = LEDMode.PIPELINE.ordinal();
  }

  private final double HEIGHT_VISION_TAPE_TO_CAMERA = Constants.FIELD.VISION_TAPE_INCHES - Constants.VISION.CAMERA_MOUNTING_HEIGHT;
  private final double CUTOFF_FREQUENCY = 100.0;
  private final double FILTER_TIME_CONSTANT = 1.0 / (2.0 * Math.PI * CUTOFF_FREQUENCY);

  private final NetworkTable table;
  private final LinearFilter filterDistance = LinearFilter.singlePoleIIR(FILTER_TIME_CONSTANT, .02);
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();
  private int seesTargetCounts = 0, noTargetCounts = 0;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setLED(LEDMode.OFF);
  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  @Override
  public void cacheSensors() {
    cache.modeLED = (int) table.getEntry("ledMode").getDouble(1.0);
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    cache.xDegrees = table.getEntry("tx").getDouble(0.0);
    cache.yDegrees = table.getEntry("ty").getDouble(0.0);
    cache.areaRaw = table.getEntry("ta").getDouble(0.0);

    seesTargetCounts++;
    noTargetCounts++;
    if (!isTargetPresent()) {
      seesTargetCounts = 0;
    } else {
      noTargetCounts = 0;
    }
    cache.distanceCalculated = calculateGroundDistanceToHubInches();
    if (isTargetPresent()) {
      cache.rpmCalculated = new ShootSetpoint(
        Constants.VISION.MAP.getRPMBottom(cache.distanceFiltered),  // TODO filter distance before calculating rpms?
        Constants.VISION.MAP.getRPMTop(cache.distanceFiltered)  // TODO filter distance before calculating rpms?
      );
    }
    // Use prior rpm calculated when we lose vision

    if (cache.seesTarget) {
      cache.distanceFiltered = filterDistance.calculate(cache.distanceCalculated);
    }
    if (noTargetCounts > 5) {
      filterDistance.reset();
    }
    // TODO should use distanceFiltered?
    cache.inShootRange = cache.distanceCalculated > Constants.VISION.MAP.getDistanceMin() && cache.distanceCalculated < Constants.VISION.MAP.getDistanceMax();
  }

  @Override
  public void updateHardware() {
    if (outputs.modeLED != cache.modeLED) {
      table.getEntry("ledMode").setNumber(outputs.modeLED);
      cache.modeLED = outputs.modeLED;
    }
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Vision: Target", cache.seesTarget);
    SmartDashboard.putNumber("Vision: Distance", cache.distanceFiltered);
    SmartDashboard.putNumber("Vision: X", cache.xDegrees);
    if (Constants.TUNING.VISION) {
      SmartDashboard.putNumber("Vision: Area", cache.areaRaw);
      SmartDashboard.putNumber("Vision: Distance Raw", cache.distanceCalculated);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    filterDistance.reset();
    setLED(LEDMode.ON);
  }

  @Override
  public void onDisable() {
    setLED(LEDMode.OFF);
  }

  public void setLED(LEDMode mode) { outputs.modeLED = mode.ordinal(); }

  public ShootSetpoint getShooterRPM() { return cache.rpmCalculated; }
  public double getTurretError() { return cache.xDegrees; }

  public boolean isTargetPresent() { return cache.seesTarget; }
  public boolean isStable() { return seesTargetCounts >= Constants.VISION.STABLE_COUNTS; }
  public boolean isInShootRange() { return cache.inShootRange; };

  /** See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#. */
  private double calculateGroundDistanceToHubInches() {
    if (isTargetPresent()) {
      return HEIGHT_VISION_TAPE_TO_CAMERA / Math.tan(Math.toRadians(cache.yDegrees + Constants.VISION.CAMERA_MOUNTING_ANGLE)) - Constants.VISION.CAMERA_MOUNTING_OFFSET;
    }
    return cache.distanceFiltered;  // Last known
  }

  @Override
  public void runTests() {
    Test.add(this, "Target", cache.seesTarget);
  }
}
