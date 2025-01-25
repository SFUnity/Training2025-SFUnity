package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;

/** Put high level commands here */
public final class RobotCommands {
  public Command setCoralIntake(Elevator elevator, Carriage rollers) {
    return elevator
        .request(ElevatorHeight.Source)
        .withDeadline(rollers.intakeCoral())
        .until(elevator::atDesiredHeight)
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Carriage rollers) {
    return elevator
        .enableElevator()
        .until(elevator::atDesiredHeight)
        .andThen(rollers.placeCoral().withTimeout(1))
        // .until(() -> rollers.coralHeld() == false)
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
