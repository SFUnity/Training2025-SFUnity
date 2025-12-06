package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double topVolts = 0.0;
    public double bottomVolts = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
