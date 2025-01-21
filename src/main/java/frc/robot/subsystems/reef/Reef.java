package frc.robot.subsystems.reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.reef.elevator.Elevator;
import frc.robot.subsystems.reef.rollers.Rollers;

public class Reef extends SubsystemBase {
  private final Rollers rollers;
  private final Elevator elevator;

  public Reef(Rollers rollers, Elevator elevator) {
    this.rollers = rollers;
    this.elevator = elevator;
  }

  public void periodic() {}

  public boolean atDesiredHeight() {
    return elevator.atDesiredHeight();
  }

  public Command setCoralIntake() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.intakeCoral())
        .withName("intakeCoral");
  }

  public Command setScoreL1() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL1");
  }

  public Command setScoreL2() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL2");
  }

  public Command setScoreL3() {
    return elevator
        .l3()
        .until(() -> atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL3");
  }

  public Command setDealgaeLow() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.lowDealgaefy())
        .withName("dealgaefyLow");
  }

  public Command setDealgaeHigh() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("dealgaefyHigh");
  }

  public Command setScoreProcessor() {
    return elevator
        .source()
        .until(() -> atDesiredHeight())
        .andThen(rollers.scoreProcessor())
        .withName("");
  }
}
