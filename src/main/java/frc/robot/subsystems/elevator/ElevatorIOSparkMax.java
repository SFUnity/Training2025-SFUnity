package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
  private final RelativeEncoder encoder = elevatorMotor.getEncoder();

  private double prevoiusPosition = 0;
  private long prevoiusTime = 0;
  private long currentTime = 0;
  private double deltaPosition = 0;
  private double deltaTime = 0;

  private final LoggedTunableNumber sillyHeightOffset =
      new LoggedTunableNumber("Elevator/sillyHeightOffset", 0.05);

  public ElevatorIOSparkMax() {
    var motorConfig = sparkConfig(false, 1);
    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    configureSpark(elevatorMotor, motorConfig, true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    prevoiusTime = currentTime;
    currentTime = System.nanoTime();

    prevoiusPosition = inputs.position;
    // how much the elevator moves per rotation (from otis)
    inputs.position = encoder.getPosition() * (wheelRadius) - sillyHeightOffset.get();
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

  @Override
  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }
}
