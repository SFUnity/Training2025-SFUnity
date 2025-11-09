package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerCurrent = 0.0;
    public double rollerVoltage = 0.0;
    public double rollerPositionRad = 0.0;
    public double pivotCurrentPositionDeg = 0;
    public double pivotAppliedVolts = 0;
    public double pivotCurrentAmps = 0;
  }

  public default void updateInput(IntakeIOInputs inputs) {}

  public default void runRollers(double volts) {}

  public default void setPivotPosition(double setpointDeg) {}
}
