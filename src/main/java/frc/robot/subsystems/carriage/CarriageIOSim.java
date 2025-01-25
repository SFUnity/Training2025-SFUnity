package frc.robot.subsystems.carriage;

public class CarriageIOSim implements CarriageIO {
  private double appliedVolts = 0.0;

  public CarriageIOSim() {}

  @Override
  public void updateInputs(CarrageIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
