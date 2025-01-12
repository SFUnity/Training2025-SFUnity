package frc.robot.subsystems.ground;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIO {
    @AutoLog
    public static class GroundIOInputs {
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double rollersAppliedVolts = 0.0;
        public double rollersCurrentAmps = 0.0;

    }

    default void updateInputs (GroundIOInputs inputs) {}

    default void setPivotPosition () {}

    default void stop() {}
}
