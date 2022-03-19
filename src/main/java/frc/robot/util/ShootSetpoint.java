package frc.robot.util;

import frc.robot.Constants;

public class ShootSetpoint {
  public static final ShootSetpoint DEFAULT = new ShootSetpoint(Constants.VISION.MAP.getRPMBottomDefault(), Constants.VISION.MAP.getRPMTopDefault());

  public final double rpmBottom, rpmTop;
  
  public ShootSetpoint() {
    this(0.0, 0.0);
  }

  public ShootSetpoint(double distance) {
    this(Constants.VISION.MAP.getRPMBottom(distance), Constants.VISION.MAP.getRPMTop(distance));
  }
  
  public ShootSetpoint(double rpmBottom, double rpmTop) {
    this.rpmBottom = rpmBottom;
    this.rpmTop = rpmTop;
  }
}
