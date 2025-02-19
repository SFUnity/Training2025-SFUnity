package frc.robot.subsystems.carriage;

import org.littletonrobotics.junction.AutoLog;

public interface CarriageIO {
  @AutoLog
  public static class CarrageIOInputs {
    public double positionRots = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean beamBreak = false;
  }

  public default void updateInputs(CarrageIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void resetEncoder() {}
}
