package frc.robot.subsystems.flywheels;

public class FlywheelsIOSim implements FlywheelsIO{
   double appliedVolts = 0.0;

    public FlywheelsIOSim() {}

@Override 
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.flywheelVoltsBottom = appliedVolts;
        inputs.flywheelVoltsTop = appliedVolts;
     }
     @Override 
     public void runFlywheelVoltsTop(int volts) {
        appliedVolts = volts;
     }
      @Override 
     public void runFlywheelVoltsBottom(int volts) {
        appliedVolts = volts;
}
}

//not done yet, i dont know if i need to change the volts in appliedVolts?