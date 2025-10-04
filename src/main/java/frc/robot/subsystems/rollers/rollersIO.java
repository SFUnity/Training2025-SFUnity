package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface rollersIO {

    @AutoLog
    public static class RollerIOInputs {   
        public double current = 0.0;
        public double voltage = 0.0;
        public double positionRad = 0.0;
    }

    public default void updateInputs {RollersIOInputs inputs} {}

    public default void runVolts{double volts} {}
}

