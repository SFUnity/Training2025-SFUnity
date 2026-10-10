package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

    @AutoLog
    public static class RollersIOinputs {
        public double volts = 0.0;
    }

    public default void UpdateInputs(RollersIOinputs inputs) {}

    public default void RunVolts() {}

}