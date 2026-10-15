package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface rollersIO {

    @AutoLog
    public static class RollerIOInputs {
        public double volts = 0.0;
       
}

    public default void updateInputs(RollerIOInputs inputs) {} 

    public default void runVolts(int volts) {

    }
}
