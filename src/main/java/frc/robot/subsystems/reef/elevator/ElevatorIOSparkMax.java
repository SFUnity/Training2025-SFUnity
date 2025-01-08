package frc.robot.subsystems.reef.elevator;

import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.PoseManager;
public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private final SparkClosedLoopController pid = elevatorMotor.getClosedLoopController();

  public ElevatorIOSparkMax() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}

  @Override
  public void setHeight(double desiredHeight) {
    pid.setReference(desiredHeight, ControlType.kPosition);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }
}
