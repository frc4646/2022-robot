package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeOpenLoop extends InstantCommand {
  private final Intake subsystem = RobotContainer.INTAKE;
  private final double intakeSpeed;

  public IntakeOpenLoop() {
    this(0.0);
  }

  public IntakeOpenLoop(double intakePercent) {
    addRequirements(subsystem);
    this.intakeSpeed = intakePercent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(intakeSpeed);
  }
}
