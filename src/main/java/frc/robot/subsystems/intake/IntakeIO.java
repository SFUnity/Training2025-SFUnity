package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotCurrentPositionDeg = 0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runPivot(double volts) {}

  public default void setPivotPosition(double setpointDeg) {}

  
}
