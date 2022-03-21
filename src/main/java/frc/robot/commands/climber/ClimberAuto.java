package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberAuto extends SequentialCommandGroup {
  private static final Climber climber = RobotContainer.CLIMBER;
  
  public ClimberAuto() {
    addRequirements(climber);
    addCommands(
      new SelectCommand(ClimberAuto::select)
    );
  }

  public static Command select() {
    // if (!climber.isZeroed()) {
    //   return new ClimberZero().withTimeout(10.0);
    // }
    return new ClimberTeleop();
  }
}
