package frc.robot.controls;

import java.util.List;

import frc.robot.subsystems.SmartSubsystem;

public class Controls {
  public final DriverControls driver;
  public final OperatorControls operator;
  public final DashboardControls dashboard;

  public Controls() {
    driver = new DriverControls();
    operator = new OperatorControls();
    dashboard = new DashboardControls();
  }
}
