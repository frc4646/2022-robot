package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeAuto extends CommandBase {
  private final Intake subsystem = RobotContainer.INTAKE;

  public IntakeAuto() {
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {
    subsystem.setOpenLoop(0.0);
  }
}
