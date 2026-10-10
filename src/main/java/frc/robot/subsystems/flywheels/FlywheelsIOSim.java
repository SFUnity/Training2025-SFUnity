package frc.robot.subsystems.flywheels;

public class FlywheelsIOSim implements FlywheelsIO{
 double appliedVolts = 0.0;

    public FlywheelsIOSim () {}

    @Override
    public void updateInputs(FlywheelsIOInputs input) {
        input.flywheelVoltsTop = appliedVolts;
        input.flywheelVoltsBottom = appliedVolts;
    }

    @Override
    public void flywheelVoltsTop(int volts){
        appliedVolts = volts;
    };
    
    @Override
    public void flywheelVoltsBottom(int volts){
        appliedVolts = volts;
    };
}

//This flywheel code does not work, I'm not sure what the error is ! But a flywheel has one wheel spinning one way, another spinning the opposite way!