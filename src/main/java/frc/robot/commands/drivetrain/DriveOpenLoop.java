package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveOpenLoop extends InstantCommand {
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
  private final double output;

  public DriveOpenLoop(double percent) {
    addRequirements(subsystem);
    this.output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setBrakeMode(true);
    subsystem.setOpenLoop(output, output);
  }
}
