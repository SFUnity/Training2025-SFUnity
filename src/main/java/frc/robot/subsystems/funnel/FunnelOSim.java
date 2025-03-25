package frc.robot.subsystems.funnel;

import frc.robot.subsystems.carriage.CarriageIO.CarrageIOInputs;

public class FunnelOSim implements FunnelIO{
    private double appliedVolts = 0.0;

  public FunnelOSim() {}

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
