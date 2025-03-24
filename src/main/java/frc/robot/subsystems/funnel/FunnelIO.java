package frc.robot.subsystems.funnel;

public interface FunnelIO {
    @AutoLog
    public static class FunnelIOInputs {
      public double appliedVolts = 0.0;
      public double currentAmps = 0.0;
    }
  
    public default void updateInputs(ElevatorIOInputs inputs) {}
  
    public default void runVolts(double volts) {}
}
