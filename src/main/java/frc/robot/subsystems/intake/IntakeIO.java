package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double current = 0.0;
    public double voltage = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInput(IntakeIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
