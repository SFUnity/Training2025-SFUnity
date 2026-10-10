package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO{
    double appliedVolts = 0.0;

    public RollersIOSim() {}

    @Override
    public void UpdateInputs(RollersIOinputs inputs) {
        inputs.volts = appliedVolts;
    }

    @Override
    public RunVolts(int volts) {
        appliedVolts = volts;
    }

}
