package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface rollersio {

    @AutoLog
    public static class RollersioInputs {
        public double current = 0.0;
        public double voltage = 0.0;
        public double positionRad = 0.0;


    }

    public default void updateInputs (RollersioInputs inputs) {}

    public default void runVolts (double volts) {}

    
    
}
