package frc.robot.subsystems.rollers; 

    public interface RollersIO {

        @AutoLog
        public static class RollersIOInputs {
            double rollerspeed = 0.0;
            double rollerVolts = 0.0;
            double rollerStatorCurrent = 0.0;
            double rollerSupllyCurrent = 0.0;
        }

        public default void updateInputs(RollersIOInputs inputs) {};

       public default void runVolts(double volts) {}; 

       public default void stop() {};
    }
