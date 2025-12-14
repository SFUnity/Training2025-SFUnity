package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerCurrent = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerPositionRad = 0.0;

        public double pivotCurrent = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotPositionRad = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void ruRollerVolts(double volts) {}

    public default void runPivotVolts(double volts) {}

    public default void setPivotPosition(double setPoinPosition) {}

    public default void resetEncoder(double position) {}
}
