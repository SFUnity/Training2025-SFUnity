package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  public static class RollersIOInputs {
    public double rollerCurrent = 0.0;
    public double voltage = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
