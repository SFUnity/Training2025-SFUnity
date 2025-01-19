package frc.robot.subsystems.ground;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class GroundIOSparkMax implements GroundIO {
    private static final double GEAR_RATIO = 1;
    private final SparkMax pivot = new SparkMax(0, null);
    private final SparkMax rollers = new SparkMax(0, null);
    private final RelativeEncoder encoder = pivot.getEncoder();
    private final SparkClosedLoopController pid = pivot.getClosedLoopController();

    @Override
    public void stop() {
        pivot.stopMotor();
    }

    @Override
    public void updateInputs(GroundIOInputs inputs) {
        inputs.pivotCurrentPosition = Units.rotationsToDegrees(encoder.getPosition() / GEAR_RATIO);
        inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
        inputs.pivotCurrentAmps =  pivot.getOutputCurrent();

        inputs.rollersAppliedVolts = rollers.getAppliedOutput() * rollers.getBusVoltage();
        inputs.rollersCurrentAmps = rollers.getOutputCurrent();

    }

    @Override
    public void runGroundRollers(double percentOutput) {
        rollers.set(percentOutput); 
    }

    @Override
    public void setPivotPosition(double setpointRots) {
        if (controllerNeedsReset) {
            controller.reset();
            controllerNeedsReset = false;
        }
    }

}
