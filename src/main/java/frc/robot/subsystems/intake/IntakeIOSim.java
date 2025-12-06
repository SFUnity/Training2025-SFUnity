package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topVolts = appliedVolts;
    inputs.bottomVolts = -appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
