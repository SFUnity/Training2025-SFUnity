package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO {
    private double aplliedVolts = 0.0;
    
    public RollersIOSim() {} 
   
    @Override
    public void updateInputs(RollersIOIinputs inputs) {
        inputs.voltage = aplliedVolts;
    }

    @Override
    public void runVolts(double volts) {
        aplliedVolts = volts;
    }   
}

