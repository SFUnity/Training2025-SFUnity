package frc.robot.subsystems.reef.rollers;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs {
      public double positionRots;
      public double velocityRotsPerSec = 0.0;
      public double appliedVolts = 0.0;
      public double[] currentAmps = new double[] {};
    }
  
    public default void updateInputs(RollersIOInputs inputs) {}
  
    public default void runVolts(double volts) {}
  
    public default void stop() {}
  }
