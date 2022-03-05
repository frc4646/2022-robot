package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;

public class ClimberAuto extends ConditionalCommand {
  public ClimberAuto() {
    super(new ClimberTeleop(), new ClimberZero(), ClimberAuto::isClimberZeroed);
  }

  private static boolean isClimberZeroed() {
    return RobotContainer.CLIMBER.IsZeroed();
  }
}
