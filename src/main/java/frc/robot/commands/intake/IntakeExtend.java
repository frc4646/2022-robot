package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeExtend extends InstantCommand {
  private final Intake subsystem = RobotContainer.INTAKE;
  public final boolean extend;

  public IntakeExtend(boolean isWantExtended) {
    this.extend = isWantExtended;
  }

  @Override
  public void initialize() {
    subsystem.setExtend(extend);
  }
}
