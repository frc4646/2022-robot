package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeJuggle extends CommandBase {
  private final Intake subsystem = RobotContainer.INTAKE;

  public double intakeSpeed = 0.0;
  public int pulse = 0;

  public IntakeJuggle() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.setOpenLoop(intakeSpeed);
    pulse++;
    intakeSpeed = Math.sin(pulse*2*Math.PI/50); // at 20ms, this should be one pulse a second
  }
}
