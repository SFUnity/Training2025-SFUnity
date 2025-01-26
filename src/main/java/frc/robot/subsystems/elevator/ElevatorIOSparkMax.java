package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private double prevoiusPosition = 0;
  private long prevoiusTime = 0;
  private long currentTime = 0;
  private double deltaPosition = 0;
  private double deltaTime = 0;

  public ElevatorIOSparkMax() {
    motorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(elevatorPIDMinInput, elevatorPIDMaxInput);

    motorConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwAverageDepth(2);
    motorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 100))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    configureSpark(elevatorMotor, motorConfig, true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    prevoiusTime = currentTime;
    currentTime = System.nanoTime();

    prevoiusPosition = inputs.position;
    inputs.position = encoder.getPosition() * .8; //how much the elevator moves per rotation (from otis)
    deltaPosition = inputs.position - prevoiusPosition;
    deltaTime = (currentTime - prevoiusTime) / 1e9;
    inputs.velocityInchesPerSec = deltaPosition / deltaTime;

    inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
    inputs.currentAmps = elevatorMotor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    elevatorMotor.setVoltage(volts);
  }
}
