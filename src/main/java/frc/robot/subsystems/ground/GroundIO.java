package frc.robot.subsystems.ground;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface GroundIO {
    @AutoLog
    public static class GroundIOInputs {
        public double pivotCurrentPosition = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double rollersAppliedVolts = 0.0;
        public double rollersCurrentAmps = 0.0;

    }

    default void updateInputs(GroundIOInputs inputs) {}

    default void runIntakeRollers(double percentOutput) {}

    default void setPivotPosition(Angle angle) {}

    default void stop() {}
}
