package frc.robot.subsystems.rollers;

public class rollersIOSim implements rollersIO {
    double appliedVolts = 0.0;

    public rollersIOSim() {}

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.volts = appliedVolts;

    }

    @Override
    public void runVolts(int volts) {
        appliedVolts = volts;
    }
    

}
    

    

