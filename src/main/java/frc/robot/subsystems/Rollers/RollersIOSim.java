package frc.robot.subsystems.Rollers;

public class RollersIOSim implements RollersIO {
  private double appliedVolts = 0.0;

  public RollersIOSim() {}

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.topVoltage = appliedVolts;
    inputs.bottomVoltage = -(appliedVolts);
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
