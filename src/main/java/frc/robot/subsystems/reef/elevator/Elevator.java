package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ProfiledPIDController profile =
      new ProfiledPIDController(
          ElevatorConstants.profileP,
          ElevatorConstants.profileI,
          ElevatorConstants.profileD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxElevatorSpeed, ElevatorConstants.maxElevatorAcceleration));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private double setpoint = 0;

  private final ElevatorIO io;
  // private final PoseManager poseManager;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io /*, PoseManager poseManager */) {
    this.io = io;
    // this.poseManager = poseManager;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void calculateDesiredAngle(double kP) {
    goal = new TrapezoidProfile.State(kP, 0);
    //TODO: is setpoint needed?
    setpoint = profile.calculate(inputs.positionRots, goal);
  }

  public void runElevator() {
    io.setHeight(goal.position);
  }

  public void stop() {
    io.stop();
  }

  public Command L1() {
    return run(() -> {
          calculateDesiredAngle(ElevatorConstants.desiredHeightL1);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredAngle(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyL1");
  }
}
