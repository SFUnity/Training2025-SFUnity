package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;

/** Put high level commands here */
public final class RobotCommands {
  public Command coralIntake(Elevator elevator, Carriage carriage) {
    return elevator
        .request(Source)
        .andThen(elevator.enableElevator())
        .until(elevator::atGoalHeight)
        .andThen(carriage.intakeCoral())
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Carriage carriage) {
    return elevator
        .enableElevator()
        .until(elevator::atGoalHeight)
        .andThen(carriage.placeCoral().withTimeout(1))
        // .until(() -> carriage.coralHeld() == false)
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
