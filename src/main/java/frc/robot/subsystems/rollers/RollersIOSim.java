package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO{
    double appliedVolts = 0.0;

    public RollersIOSim () {}

    @Override
    public void updateInputs(RollersIOInputs input) {
        input.rollerVolts = appliedVolts; 
    }

    @Override
    public void runVolts(int volts){
        appliedVolts = volts;
    };

}
