package frc.robot.subsystems.carrage;

public class CarrageIOSim implements CarrageIO {
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
