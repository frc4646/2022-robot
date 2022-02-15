package frc.robot;

import frc.robot.commands.CompressorAuto;
import frc.robot.commands.OnDisabledDelayed;
import frc.robot.commands.SignalDriveTeam;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.controls.AutoModeSelector;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;
import frc.robot.util.Test;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.subsystems.Turret;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final double TIME_CLIMBER_REQUIRED_TO_HOLD = 5.0;
  private final double TIME_ON_DISABLE_DELAYED = TIME_CLIMBER_REQUIRED_TO_HOLD * 2.0;

  public static Agitator AGITATOR;
  public static Climber CLIMBER;
  public static Diagnostics DIAGNOSTICS;
  public static Drivetrain DRIVETRAIN;
  public static Feeder FEEDER;
  public static Hood HOOD;
  public static Intake INTAKE;
  public static Infrastructure INFRASTRUCTURE;
  public static Shooter SHOOTER;
  public static Turret TURRET;
  public static Vision VISION;
  private final List<SmartSubsystem> allSubsystems;

  public static Controls CONTROLS;

  public final AutoModeSelector autoModeSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AGITATOR = new Agitator();
    // CLIMBER = new Climber();
    DIAGNOSTICS = new Diagnostics();
    DRIVETRAIN = new Drivetrain();
    FEEDER = new Feeder();
    HOOD = new Hood();
    INFRASTRUCTURE = new Infrastructure();
    INTAKE = new Intake();
    SHOOTER = new Shooter();
    // TURRET = new Turret();
    VISION = new Vision();
    allSubsystems = Arrays.asList(AGITATOR, DRIVETRAIN, FEEDER, HOOD, INFRASTRUCTURE, INTAKE, SHOOTER, /*TURRET,*/ VISION, DIAGNOSTICS);

    CONTROLS = new Controls();  // Create after subsystems
    DRIVETRAIN.setDefaultCommand(new DriveTeleop());
    INFRASTRUCTURE.setDefaultCommand(new CompressorAuto());
    DIAGNOSTICS.setDefaultCommand(new SignalDriveTeam());

    autoModeSelector = new AutoModeSelector();

    // TODO TURRET.forceZero();
  }

  public void cacheSensors() {
    allSubsystems.forEach(SmartSubsystem::cacheSensors);
  }

  public void updateDashboard() {
    allSubsystems.forEach(SmartSubsystem::updateDashboard);
  }

  public void onEnable(boolean isAutonomous) {
    for(SmartSubsystem subsystem : allSubsystems) {
      subsystem.onEnable(isAutonomous);
    }
  }

  public void onDisable() {
    allSubsystems.forEach(SmartSubsystem::onDisable);
    new OnDisabledDelayed().withTimeout(TIME_ON_DISABLE_DELAYED).schedule();
  }

  public void runTests() {
    VISION.setLED(LEDMode.BLINK);
    Timer.delay(3.0);
    Test.reset();
    allSubsystems.forEach(SmartSubsystem::runTests);
    Test.results();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
