package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.units.measure.Angle;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax pivot = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax rollers = new SparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkClosedLoopController pid = pivot.getClosedLoopController();

  public IntakeIOSparkMax() {
    var pivotConfig = sparkConfig(pivotInverted, pivotPositionFactor);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(intakePIDMinInput, intakePIDMaxInput);
    configureSpark(pivot, pivotConfig, true);

    var rollerConfig = sparkConfig(rollersInverted, rollersPositionFactor);
    configureSpark(rollers, rollerConfig, true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotCurrentPosition = Rotations.of(encoder.getPosition());
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();

    inputs.rollersAppliedVolts = rollers.getAppliedOutput() * rollers.getBusVoltage();
    inputs.rollersCurrentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void runRollers(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(Angle setpointRots) {
    pid.setReference(setpointRots.in(Degrees), ControlType.kPosition);
  }
}
