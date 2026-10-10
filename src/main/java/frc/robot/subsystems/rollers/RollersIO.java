package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

    @AutoLog
    public static class RollersIOInputs {
       public double rollerSpeed = 0.0;
       public double rollerVolts = 0.0;
       public double rollerStatorCurrent = 0.0;
       public double rollerSupplyCurrent = 0.0;
    }

    public default void updateInputs(RollersIOInputs input) {};

    public default void runVolts(int volts) {};

    public default void stop() {};

}
