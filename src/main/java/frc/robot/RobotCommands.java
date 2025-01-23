package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.rollers.Rollers;

/** Put high level commands here */
public final class RobotCommands {
  public Command setCoralIntake(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.intakeCoral())
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Rollers rollers) {
    return elevator
        .enableElevator()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .until(() -> rollers.coralHeld() == false)
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
