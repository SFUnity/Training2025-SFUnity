package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInput(IntakeIOInputs inputs) {
    inputs.voltage = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
