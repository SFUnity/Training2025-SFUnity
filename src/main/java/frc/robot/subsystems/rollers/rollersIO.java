package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public double example_input = 0.0;
    }
    
    public default void updateInputs(RollersIOInputs inputs){}

    public default void exampleMethod(double example_input) {}
}