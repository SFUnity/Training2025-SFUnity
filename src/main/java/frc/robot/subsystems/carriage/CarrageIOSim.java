package frc.robot.subsystems.carriage;

public class CarrageIOSim implements CarriageIO {
  private double appliedVolts = 0.0;

  public CarrageIOSim() {}

  @Override
  public void updateInputs(CarrageIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
