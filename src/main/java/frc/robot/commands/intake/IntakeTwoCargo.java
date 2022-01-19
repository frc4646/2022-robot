package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeTwoCargo extends CommandBase {
  private Intake subsystem = RobotContainer.INTAKE;

  public IntakeTwoCargo() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // TODO ask how many cargo in indexer
    // TODO maybe figure out if subsystem is jammed and do something like pulse it in reverse?
    // TODO need timer to debounce?
    // TODO should we have a state machine?
    // TODO set motor
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setIntakeSpeed(); // TODO subsystem.setOpenLoop(0.0);
  }
}
