package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberAuto extends SelectCommand {
    public ClimberAuto() {
      super(ClimberAuto::nextCommand);
    }

  private static Command nextCommand() {
    return new ClimberTeleop();
    // if (climber.hasBeenZeroed()) {
    //   return new ClimberTeleop();
    // }
    // return new ClimberZero();
  }
}
