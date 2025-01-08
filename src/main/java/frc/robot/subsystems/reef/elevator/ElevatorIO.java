package frc.robot.subsystems.reef.elevator;

import org.littletonrobotics.junction.AutoLog;
public interface ElevatorIO{
    @AutoLog
    public static class ElevatorIOInputs{
        public double positionRots = 0.0;
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }
    public default void updateInputs(ElevatorIOInputs inputs){ }
    
}

//Calculate
//updateInputs
//setHeight
//setPID
//setFF