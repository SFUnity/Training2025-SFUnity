package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public class FlywheelsIO {
 
    @AutoLog
    public static class RollersIOInputs {
        public double volts = 0.0;

    }

    public default void UpdateInputs(FlywheelsIOInputs inputs) { }

    public default void RunVolts(int volts) {}
}
