package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO {
  private double appliedVolts = 0.0;

  public RollersIOSim() {}

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
