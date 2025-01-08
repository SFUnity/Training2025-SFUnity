package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Elevator {
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxElevatorSpeed, ElevatorConstants.maxElevatorAcceleration));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public Elevator() {}

  private void calculateDesiredAngle(double kP) {
    goal = new TrapezoidProfile.State(kP, 0);

    setpoint = profile.calculate(kP, setpoint, goal);
  }
}
