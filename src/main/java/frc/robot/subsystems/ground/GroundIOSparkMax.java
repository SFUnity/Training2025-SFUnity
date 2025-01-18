package frc.robot.subsystems.ground;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class GroundIOSparkMax implements GroundIO {
    private final SparkMax pivot = new SparkMax(0, null);
    private final SparkMax rollers = new SparkMax(0, null);
    private final RelativeEncoder encoder = pivot.getEncoder();
    private final SparkClosedLoopController pid = pivot.getClosedLoopController();
}
