package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeActivate extends CommandBase {
  private final IntakeSubsystem subsystem = RobotContainer.INTAKE;

  public IntakeActivate() {
      addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setIntakeSpeed();
  }
}
