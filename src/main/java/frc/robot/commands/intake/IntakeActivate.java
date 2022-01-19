package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeActivate extends InstantCommand {
  private final Intake subsystem = RobotContainer.INTAKE;

  public IntakeActivate() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setIntakeSpeed();
  }
}
