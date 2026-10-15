package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public class RollersIO {

    @AutoLog 
        public static class RollersIOInputs {
        double rollerSpeed = 0.0;
        double rollerVolts = 0.0; 
        double rollerStatorCurrent = 0.0;
        double rollerSupplyCurrent = 0.0; 

    } 

    public void updateInputs (RollersIOInputs inputs) {};

    public void runVolts(double volts) {};

    public void stop() {};

}
