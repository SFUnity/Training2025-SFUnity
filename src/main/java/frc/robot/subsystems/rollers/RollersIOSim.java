package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO {
    private double rollerSpeed = 0.0;
    private double rollerVolts = 0.0;
    private double rollerStatorCurrent = 0.0;
    private double rollerSupplyCurrent = 0.0;

    public RollersIOSim() {
        
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        inputs.rollerSpeed = this.rollerSpeed;
        inputs.rollerVolts = this.rollerVolts;
        inputs.rollerStatorCurrent = this.rollerStatorCurrent;
        inputs.rollerSupplyCurrent = this.rollerSupplyCurrent;
    }

    @Override
    public void runVolts(double volts) {
        this.rollerVolts = volts;
    }

    @Override
    public void stop() {
        this.rollerVolts = 0.0;
    }
}
