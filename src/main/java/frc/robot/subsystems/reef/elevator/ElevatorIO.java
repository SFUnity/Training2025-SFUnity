package frc.robot.subsystems.reef.elevator;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.units.measure.Angle;
public interface ElevatorIO{
    @AutoLog
    public static class ElevatorIOInputs{
        public double positionRots = 0.0;
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }
    public default void updateInputs(ElevatorIOInputs inputs){ }
    

    public default void setHeight(double desiredHeight) {}

    public default void calculateDesiredAngle(double kP) {}
}

//Calculate
//updateInputs
//setHeight
//setPID
//setFF