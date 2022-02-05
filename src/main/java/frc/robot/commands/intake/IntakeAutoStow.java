package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class IntakeAutoStow extends CommandBase {
  private Intake subsystem = RobotContainer.INTAKE;
  private Drivetrain drive = RobotContainer.DRIVETRAIN;

  public IntakeAutoStow() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(drive.getRPM()) > .75) {
      subsystem.setExtend(false);
    }
  }
}
