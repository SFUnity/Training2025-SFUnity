package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.rollers.Rollers;

/** Put high level commands here */
public final class RobotCommands {
  public Command setCoralIntake(Elevator elevator, Rollers rollers) {
    return elevator
        .request(ElevatorHeight.Source)
        .withDeadline(rollers.intakeCoral())
        .until(elevator::atDesiredHeight)
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Rollers rollers) {
    return elevator
        .enableElevator()
        .andThen(new WaitUntilCommand(() -> elevator.atDesiredHeight()))
        .andThen(rollers.placeCoralAndHighDealgify())
        // .until(() -> rollers.coralHeld() == false)
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
