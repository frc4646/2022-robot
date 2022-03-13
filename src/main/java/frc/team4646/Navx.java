package frc.team4646;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class Navx {
  private final AHRS gyro;
  private Rotation2d offset;

  public Navx() {
    gyro = new AHRS();
    reset();
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d().rotateBy(offset);
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void reset() {
    Rotation2d current = gyro.getRotation2d();
    offset = current.rotateBy(Rotation2d.fromDegrees(-current.getDegrees() * 2.0));
  }
}
