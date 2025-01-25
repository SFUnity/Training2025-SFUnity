package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carrage.Carrage;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;

/** Put high level commands here */
public final class RobotCommands {
  public Command setCoralIntake(Elevator elevator, Carrage rollers) {
    return elevator
        .request(ElevatorHeight.Source)
        .withDeadline(rollers.intakeCoral())
        .until(elevator::atDesiredHeight)
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Carrage rollers) {
    return elevator
        .enableElevator()
        .until(elevator::atDesiredHeight)
        .andThen(rollers.placeCoralAndHighDealgify().withTimeout(1))
        // .until(() -> rollers.coralHeld() == false)
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
