package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  public static class RollersIOInputs {
    public double topCurrent = 0.0;
    public double topVoltage = 0.0;
    public double positionRad = 0.0;

    public double bottomCurrent = 0.0;
    public double bottomVoltage = 0.0;
    public double bottomPositionRad = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
