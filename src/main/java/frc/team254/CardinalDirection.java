package frc.team254;

import edu.wpi.first.math.geometry.Rotation2d;

public enum CardinalDirection {
  BACK(180),
  FRONT(0),
  LEFT(45, 90),
  RIGHT(-45, -90),
  NONE(0),
  FRONT_LEFT(45, 45),
  FRONT_RIGHT(-45, -45),
  BACK_LEFT(-45, 135),
  BACK_RIGHT(45, 235);

  private static final CardinalDirection[] directions = CardinalDirection.values();
  public final Rotation2d rotation;
  private final Rotation2d inputDirection;

  CardinalDirection(double degrees) {
    this(degrees, degrees);
  }

  CardinalDirection(double degrees, double inputDirectionDegrees) {
    rotation = Rotation2d.fromDegrees(degrees);
    inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
  }

  public static CardinalDirection findClosest(double xAxis, double yAxis) {
    return findClosest(new Rotation2d(yAxis, -xAxis));
  }

  public static CardinalDirection findClosest(Rotation2d stickDirection) {
    CardinalDirection closest = null;
    double closestDistance = Double.MAX_VALUE;
    for (int i = 0; i < directions.length; i++) {
      var checkDirection = directions[i];
      var distance = Math.abs(stickDirection.rotateBy(Rotation2d.fromDegrees(180)).rotateBy(checkDirection.inputDirection).getRadians());
      if (distance < closestDistance) {
        closestDistance = distance;
        closest = checkDirection;
      }
    }
    return closest;
  }

  public static boolean isDiagonal(CardinalDirection cardinal) {
    return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
  }

  public Rotation2d getRotation() {
    return rotation;
  }
}
