package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax pivot = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax rollers = new SparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkClosedLoopController pid = pivot.getClosedLoopController();

  public IntakeIOSparkMax() {
    var pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    pivotConfig.encoder.positionConversionFactor(pivotPositionFactor).velocityConversionFactor(pivotPositionFactor).uvwAverageDepth(2);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pidf(kP.get(), 0.0, 0, 0.0);
    pivotConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    configureSpark(pivot, pivotConfig, true);

    var rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    rollerConfig.encoder.positionConversionFactor(rollersPositionFactor).velocityConversionFactor(rollersPositionFactor).uvwAverageDepth(2);
    rollerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
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
