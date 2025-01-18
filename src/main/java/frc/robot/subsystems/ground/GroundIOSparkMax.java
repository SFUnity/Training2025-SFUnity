package frc.robot.subsystems.ground;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class GroundIOSparkMax implements GroundIO {
    private final PWMSparkMax leader = new PWMSparkMax(0);
    private final PWMSparkMax follower = new PWMSparkMax(0);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkClosedLoopController pid = leader.getPIDController();
}
