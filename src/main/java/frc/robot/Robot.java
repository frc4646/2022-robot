package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.TestAuto;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private double timeInitDisabled = Double.NaN;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void disabledInit() {
    // robotContainer.autoModeSelector.reset();
    // robotContainer.autoModeSelector.update();
    timeInitDisabled = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand= new TestAuto();  // TODO Use auto chooser
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();  // If we want the auto command to continue until interrupted by another command, comment this line out.
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating. */
  @Override
  public void robotPeriodic() {
    robotContainer.cacheSensors();
    CommandScheduler.getInstance().run();  // Must be called from robotPeriodic(). Runs these steps: Polls buttons, adds newly-scheduled commands, runs already-scheduled commands, removes finished or interrupted commands, calls subsystem periodic() methods.
    robotContainer.updateDashboard();
  }

  @Override
  public void disabledPeriodic() {
    // robotContainer.autoModeSelector.update();

    // Optional<Command> autoMode = robotContainer.autoModeSelector.getAutoMode();
    // if (autoMode.isPresent()) {
    //   System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
    //   autonomousCommand = autoMode.get();
    // }

    // if ((Timer.getFPGATimestamp() - timeInitDisabled) > 5.0 && (Timer.getFPGATimestamp() - timeInitDisabled) < 5.5) {
    //   System.out.println("Releasing climber!");
    //   robotContainer.CLIMBER.setBrakeMode(false);
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
