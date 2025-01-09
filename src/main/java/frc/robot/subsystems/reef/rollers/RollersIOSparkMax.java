package frc.robot.subsystems.reef.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class RollersIOSparkMax implements RollersIO {
  private final SparkMax rollerMotor =
      new SparkMax(RollersConstants.rollerMotorID, MotorType.kBrushless);
  private final RelativeEncoder encoder = rollerMotor.getEncoder();

  public RollersIOSparkMax() {}

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps =
        new double[] {rollerMotor.getOutputCurrent(), rollerMotor.getOutputCurrent()};
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }
}
