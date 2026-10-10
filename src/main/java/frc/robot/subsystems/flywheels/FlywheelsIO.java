package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {

    @AutoLog
    public static class FlywheelsIOInputs {
       public double flywheelVoltsTop = 0.0;
       public double flywheelVoltsBottom = 0.0;
    }
    public default void updateInputs(FlywheelsIOInputs inputs) {}

    public default void runFlywheelVoltsTop(int volts) {};

    public default void runFlywheelVoltsBottom(int volts) {};
}


