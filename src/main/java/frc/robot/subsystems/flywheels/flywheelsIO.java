package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;


public interface flywheelsIO {

    @AutoLog
    public static class flywheelsIOInputs {
    public double topVolts = 0.0;
    public double bottomVolts = 0.0;
    }

    public default void updateInputs(flywheelsIOInputs inputs) {}

    

   
}

