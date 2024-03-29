package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.VisionLED;
import frc.robot.commands.turret.TurretPosition;
import frc.robot.subsystems.Vision.LEDMode;

public class ClimbMode extends ConditionalCommand {
  public ClimbMode() {
    super(new SetMode(LEDMode.OFF, true), new SetMode(LEDMode.ON, false), ClimbMode::nextMode);
  }
  
  private static class SetMode extends ParallelCommandGroup {
    public SetMode(LEDMode led, boolean extend) {
      addCommands(        
        new VisionLED(led),
        new TurretPosition(0.0).withTimeout(2.0)
        // new ClimberExtend(extend),
        // new IntakeExtend(extend)
        // TODO set drivetrain brake mode?
      );
    }
  }

  private static boolean nextMode() {
    boolean next = !RobotContainer.CLIMBER.isInClimbMode();
    RobotContainer.CLIMBER.setClimbMode(next);
    return next;
  }
}
