package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Angle pivotCurrentPosition = Rotations.zero();
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double rollerVelocityRPM = 0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runRollers(double volts) {}

  default void setPivotPosition(Angle angle) {}
}
