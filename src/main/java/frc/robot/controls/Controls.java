package frc.robot.controls;

public class Controls {
  private final DriverControls driver;
  private final OperatorControls operator;
  private final DashboardControls dashboard;

  public Controls() {
    driver = new DriverControls();
    operator = new OperatorControls();
    dashboard = new DashboardControls();
  }

  public void configureButtons() {
    operator.configureButtons();
    dashboard.configureButtons();
  }

  public DriverControls getDriver() {
    return driver;
  }

  public OperatorControls getOperator() {
    return operator;
  }
}
