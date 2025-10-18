package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  public static class RollersIOInputs {
    public double current = 0.0;
    public double voltage = 0.0;
    public double opositeVoltage = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
