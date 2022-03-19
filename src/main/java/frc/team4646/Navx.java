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
    return getHeadingRaw().rotateBy(offset);
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getHeadingRate() {
    return gyro.getRate();
  }

  public void reset() {
    Rotation2d current = getHeadingRaw();
    offset = current.rotateBy(Rotation2d.fromDegrees(-current.getDegrees() * 2.0));
  }

  private Rotation2d getHeadingRaw() {
    return Rotation2d.fromDegrees(-gyro.getRotation2d().getDegrees());
  }
}
