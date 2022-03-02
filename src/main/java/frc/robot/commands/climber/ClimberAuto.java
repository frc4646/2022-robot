package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberAuto extends ConditionalCommand {
  public ClimberAuto() {
    super(
      new ClimberTeleop(),
      new ClimberZero().withTimeout(Constants.Climber.TIMEOUT_ZERO),
      () -> {return true; }
      //RobotContainer.CLIMBER::hasBeenZeroed TODO test auto zeroing
    );
  }

  private static class ClimberTeleop extends CommandBase {
    private final Climber subsystem = RobotContainer.CLIMBER;

    public ClimberTeleop() {
      addRequirements(subsystem);
    }

    @Override
    public void execute() {
      double setpoint = 0.0;
      double stick = RobotContainer.CONTROLS.operator.getClimberStick();

      if (Math.abs(stick) > Constants.Climber.DEADBAND) {
        setpoint = stick;
      }
      subsystem.setOpenLoop(setpoint);
    }
  }

  private static class ClimberZero extends CommandBase {
    private final Climber subsystem = RobotContainer.CLIMBER;

    public ClimberZero() {
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      subsystem.setSoftLimitsEnabled(false);
      subsystem.setOpenLoop(-Constants.Climber.OPEN_LOOP_ZERO);
    }

    @Override
    public boolean isFinished() {
      return subsystem.atHomingLocation();
    }
  
    @Override
    public void end(boolean interrupted) {
      subsystem.zeroSensors();
      subsystem.setSoftLimitsEnabled(true);
      subsystem.setOpenLoop(0.0);
    }
  }
}
