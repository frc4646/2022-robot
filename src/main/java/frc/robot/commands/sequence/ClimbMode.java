package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.ClimberArms;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.turret.TurretPosition;
import frc.robot.commands.vision.VisionLED;
import frc.robot.subsystems.Vision.LEDMode;

public class ClimbMode extends ConditionalCommand {
  public ClimbMode() {
    super(new SetMode(LEDMode.OFF, true), new SetMode(LEDMode.ON, false), ClimbMode::nextMode);
  }
  
  private static class SetMode extends ParallelCommandGroup {
    public SetMode(LEDMode led, boolean extend) {
      addCommands(        
        new VisionLED(led),
        new TurretPosition(Constants.TURRET.SERVO.kHomePosition, 0.1).withTimeout(1.0),  // TODO bigger tolerance instead of timeout?
        new ClimberArms(extend),
        new IntakeExtend(extend)
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
