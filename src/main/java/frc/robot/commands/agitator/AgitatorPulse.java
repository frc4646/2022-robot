package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.team4646.DelayedBoolean;

public class AgitatorPulse extends CommandBase {
  private final Agitator subsystem = RobotContainer.AGITATOR;
  private final DelayedBoolean switchTimer;
  private final double strength;
  private boolean isLeft = true;

  public AgitatorPulse(double strength, double time) {
    addRequirements(subsystem);
    switchTimer = new DelayedBoolean(time);
    this.strength = strength;
  }

  @Override
  public void initialize() {
    switchTimer.reset();
  }

  @Override
  public void execute() {
    if (switchTimer.isSet(true)) {
      isLeft = !isLeft;
      switchTimer.reset();
    }
    double setpointL = isLeft ? strength : strength * 0.25;
    double setpointR = isLeft ? strength * 0.25 : strength;
    subsystem.setOpenLoop(setpointL, setpointR);
  }
}
