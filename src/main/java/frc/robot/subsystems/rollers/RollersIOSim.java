package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO {
  double appliedVolts = 0.0;

  public RollersIOSim() {}

  @Override
  public void UpdateInputs(RollersIOInputs inputs) {
    inputs.volts = appliedVolts;
  }

  @Override
  public void RunVolts(int volts) {
    appliedVolts = volts;
  }
}
