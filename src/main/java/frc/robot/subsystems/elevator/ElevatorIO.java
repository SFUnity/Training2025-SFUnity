package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    private static final DistanceUnit Inches = null;

    public Distance position = Distance.ofBaseUnits(0, Inches);

    public LinearVelocity velocityMetersPerSec = LinearVelocity.ofBaseUnits(0, InchesPerSecond);
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
