package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.reef.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.PoseManager;
import frc.robot.util.PoseManager;

public class Elevator extends SubsystemBase{
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxElevatorSpeed, ElevatorConstants.maxElevatorAcceleration));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final ElevatorIO io;
  private final PoseManager poseManager;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
  }



  private void calculateDesiredAngle(double kP) {
    goal = new TrapezoidProfile.State(kP, 0);

    setpoint = profile.calculate(kP, setpoint, goal);
  }
}
