package frc.robot.subsystems.reef.elevator;

import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private final SparkClosedLoopController pid = elevatorMotor.getClosedLoopController();

  public ElevatorIOSparkMax() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
    inputs.currentAmps =
        new double[] {elevatorMotor.getOutputCurrent(), elevatorMotor.getOutputCurrent()};
  }

  @Override
  public void setHeight(double desiredHeight) {
    pid.setReference(desiredHeight, ControlType.kPosition);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }
}
