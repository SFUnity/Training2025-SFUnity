package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOinputs {
    public double current = 0.0;
    public double voltage = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInputs(RollersIOinputs inputs) {}

  public default void runVolts(double volts) {}
}
