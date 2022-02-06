package frc.robot.controls;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.sequence.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.SmartSubsystem;

public class DashboardControls {
  public DashboardControls() {
    // TODO if not at event/competition

    addTuningCommands();

    addSubsystem(RobotContainer.SHOOTER, new ShooterTune(), new ShooterOpenLoop(.5));
    addSubsystem(RobotContainer.INTAKE, new IntakeExtend(true));
    addSubsystem(RobotContainer.FEEDER, new FeederOpenLoop(.5));
    addSubsystem("Sequence", new ShootOpenLoop(), new DeployIntake(), new LoadCargo(), new StowIntake());
  }

  public void addTuningCommands() {
    SmartDashboard.putNumber("tuning/shooterVel", 0);
    SmartDashboard.putData("tuning/setSetpoints", new ShooterTune());
  }

  public ShuffleboardLayout addSubsystem(SmartSubsystem subsystem, CommandBase... cmds) {
    ShuffleboardLayout layout = Shuffleboard.getTab("Commands")
      .getLayout(subsystem.getName(), BuiltInLayouts.kList)
      .withSize(2, 6)
      .withProperties(Map.of("Label position", "HIDDEN"));

      layout.add(subsystem);

      for (CommandBase cmd : cmds) {
        layout.add(cmd);
      }

      return layout;
  }

  public ShuffleboardLayout addSubsystem(String name, CommandBase... cmds) {
    ShuffleboardLayout layout = Shuffleboard.getTab("Commands")
      .getLayout(name, BuiltInLayouts.kList)
      .withSize(2, 6)
      .withProperties(Map.of("Label position", "HIDDEN"));

      for (CommandBase cmd : cmds) {
        layout.add(cmd);
      }

      return layout;
  }

}
