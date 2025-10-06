package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotCurrent = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotPositionRads = 0.0;

    public double rollerCurrent = 0.0;
    public double rollerVoltage = 0.0;

    public boolean beambreak = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runRollers(double volts) {}

  default void runPivot(double volts) {}

  default void setPivotPosition(double setpointDeg) {}

  default void resetEncoder(double position) {}
}
