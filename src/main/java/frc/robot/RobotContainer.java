package frc.robot;

import frc.robot.commands.CompressorAuto;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.controls.AutoModeSelector;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.subsystems.Turret;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static Agitator AGITATOR;
  public static Climber CLIMBER;
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
    DRIVETRAIN = new Drivetrain();
    FEEDER = new Feeder();
    HOOD = new Hood();
    INFRASTRUCTURE = new Infrastructure();
    INTAKE = new Intake();
    SHOOTER = new Shooter();
    TURRET = new Turret();
    VISION = new Vision();
    allSubsystems = Arrays.asList(AGITATOR, DRIVETRAIN, FEEDER, HOOD, INFRASTRUCTURE, INTAKE, SHOOTER, TURRET, VISION);

    CONTROLS = new Controls();  // Create after subsystems
    DRIVETRAIN.setDefaultCommand(new DriveTeleop());
    INFRASTRUCTURE.setDefaultCommand(new CompressorAuto());

    autoModeSelector = new AutoModeSelector();
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
  }

  public void runTests() {
    allSubsystems.forEach(SmartSubsystem::runTests);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
