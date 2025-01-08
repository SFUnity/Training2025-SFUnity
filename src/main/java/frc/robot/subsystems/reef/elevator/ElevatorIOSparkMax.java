package frc.robot.subsystems.reef.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOSparkMax implements ElevatorIO{
    private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
         private TrapezoidProfile.State goal = new TrapezoidProfile.State();
         private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    
         public ElevatorIOSparkMax() {
        

    }
}
