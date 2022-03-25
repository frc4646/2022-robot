package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.team4646.DelayedBoolean;

public class AgitatorPulse extends CommandBase {
  private final Agitator subsystem = RobotContainer.AGITATOR;
  private final DelayedBoolean switchTimer;
  private final double strength;
  private final double strength2;
  private boolean isLeft = true;

  public AgitatorPulse() {
    this(Constants.AGITATOR.OPEN_LOOP_LOAD * 1.5, 0.5);
  }

  public AgitatorPulse(double strength, double time) {
    addRequirements(subsystem);
    switchTimer = new DelayedBoolean(time);
    this.strength = strength;
    this.strength2 = strength * 0.5;  // 0% causes cargo to pop out
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
    double setpointL = isLeft ? strength : strength * strength2;
    double setpointR = isLeft ? strength * strength2 : strength;
    subsystem.setOpenLoop(setpointL, setpointR);
  }
}
