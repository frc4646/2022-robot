package frc.robot.controls;

public class Controls {
  public final DriverControls driver;
  public final OperatorControls operator;
  public final DashboardControls dashboard;

  public Controls() {
    driver = new DriverControls();
    operator = new OperatorControls();
    dashboard = new DashboardControls();
  }

  public void configureButtons() {
    operator.configureButtons();
  }
}
