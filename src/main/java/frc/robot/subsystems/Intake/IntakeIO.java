package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface IntakeIO {
    
    @AutoLog
    public static class RollersIOInputs {
        public double current = 0.0;
        public double voltage = 0.0;
        public double positionRad = 0.0;
    }

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void runVolts(RollersIOInputs inputs) {}
}
