package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeActivate extends InstantCommand {
  private final Intake subsystem = RobotContainer.INTAKE;

  public final double intakeSpeed;

  public IntakeActivate(double intakePercent) {
    addRequirements(subsystem);
    this.intakeSpeed = intakePercent;
  }

  @Override
  public void initialize() {
    subsystem.setIntakeSpeed(intakeSpeed);
  }
}
