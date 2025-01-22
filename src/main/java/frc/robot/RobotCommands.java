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

  public Command setScoreL1(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL1");
  }

  public Command setScoreL2(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL2");
  }

  public Command setScoreL3(Elevator elevator, Rollers rollers) {
    return elevator
        .l3()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL3");
  }

  public Command setDealgaeLow(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.lowDealgaefy())
        .withName("dealgaefyLow");
  }

  public Command setDealgaeHigh(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("dealgaefyHigh");
  }

  public Command setScoreProcessor(Elevator elevator, Rollers rollers) {
    return elevator
        .source()
        .until(() -> elevator.atDesiredHeight())
        .andThen(rollers.scoreProcessor())
        .withName("");
  }
}
