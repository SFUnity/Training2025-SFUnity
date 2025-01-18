package frc.robot.subsystems.ground;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class GroundIOSparkMax implements GroundIO {
    private final PWMSparkMax leader = new PWMSparkMax();
    private final PWMSparkMax follower = new PWMSparkMax();
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkClosedLoopController pid = leader.getPIDController();
}
