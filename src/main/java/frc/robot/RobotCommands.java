package frc.robot;

import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.AlgaeHigh;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.AlgaeLow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.elevator.Elevator;

/** Put high level commands here */
public final class RobotCommands {
  public static Command score(Elevator elevator, Carriage carriage) {
    return elevator
        .enableElevator()
        .until(elevator::atGoalHeight)
        .andThen(carriage.placeCoral())
        .andThen(elevator.disableElevator())
        .withName("score");
  }

  public static Command dealgify(Elevator elevator, Carriage carriage, boolean high) {
    return elevator
        .request(high ? AlgaeHigh : AlgaeLow)
        .andThen(elevator.enableElevator().until(elevator::atGoalHeight))
        .alongWith(high ? carriage.highDealgify() : carriage.lowDealgify())
        .withName("dealgify");
  }
}
