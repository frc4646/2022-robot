package frc.robot.util;

import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;

public class ShootMap {
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
    mapBottom = new InterpolatingTreeMap<>(),
    mapTop = new InterpolatingTreeMap<>();
  private double distanceMin = Double.MAX_VALUE;
  private double distanceMax = Double.MIN_VALUE;
  private double rpmBottomMin = Double.MAX_VALUE;
  private double rpmTopMin = Double.MAX_VALUE;

  public void add(double distance, double rpmBottom, double rpmTop) {
    mapBottom.put(new InterpolatingDouble(distance), new InterpolatingDouble(rpmBottom));
    mapTop.put(new InterpolatingDouble(distance), new InterpolatingDouble(rpmTop));
    updateMinMax(distance, rpmBottom, rpmTop);
  }

  public double getRPMBottom(double distance) { return mapBottom.getInterpolated(new InterpolatingDouble(distance)).value; }
  public double getRPMTop(double distance) { return mapTop.getInterpolated(new InterpolatingDouble(distance)).value; }
  public double getDistanceMin() { return distanceMin; }
  public double getDistanceMax() { return distanceMax; }
  public double getRPMBottomMin() { return rpmBottomMin; }
  public double getRPMTopMin() { return rpmTopMin; }

  private void updateMinMax(double distance, double rpmBottom, double rpmTop) {
    if (distance < distanceMin) {
      distanceMin = distance;
    }
    if (distance > distanceMax) {
      distanceMax = distance;
    }
    if (rpmBottom < rpmBottomMin) {
      rpmBottomMin = rpmBottom;
    }
    if (rpmTop < rpmTopMin) {
      rpmTopMin = rpmTop;
    }
  }
}
