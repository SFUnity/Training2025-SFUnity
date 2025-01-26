package frc.robot;

import static frc.robot.RobotCommands.ScoreState.*;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.ground.Ground;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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

  public static ScoreState scoreState = ProcessorBack;

  public static Command fullScore(
      Drive drive,
      Elevator elevator,
      Carriage carriage,
      Ground ground,
      PoseManager poseManager,
      BooleanSupplier dealgifyAfterPlacing) {
    return drive
        .fullAutoDrive(goalPose(poseManager))
        .alongWith(
            Commands.select(
                Map.of(
                    LeftBranch,
                    new WaitUntilCommand(
                            () ->
                                poseManager.getDistanceTo(goalPose(poseManager).get())
                                    < ElevatorConstants.subsystemExtentionLimit)
                        .andThen(
                            score(elevator, carriage)
                                .alongWith(Commands.print("Running elevator and carriage score")))
                        .andThen(
                            dealgifyAfterPlacing.getAsBoolean()
                                ? Commands.runOnce(() -> scoreState = Dealgify)
                                    .andThen(
                                        fullScore(
                                            drive,
                                            elevator,
                                            carriage,
                                            ground,
                                            poseManager,
                                            () -> false))
                                : Commands.none()),
                    Dealgify,
                    dealgify(elevator, carriage, poseManager.closestFace().highAlgae),
                    ProcessorFront,
                    carriage.scoreProcessor(),
                    ProcessorBack,
                    ground.poopCmd().until(() -> !ground.algaeHeld())),
                () -> scoreState == RightBranch ? LeftBranch : scoreState))
        .withName("Score/Dealgify");
  }

  public static Supplier<Pose2d> goalPose(PoseManager poseManager) {
    return () -> {
      switch (scoreState) {
        case LeftBranch:
          return apply(poseManager.closestFace().leftBranch.pose);
        case RightBranch:
          return apply(poseManager.closestFace().rightBranch.pose);
        case Dealgify:
          return apply(poseManager.closestFace().pose);
        case ProcessorFront:
          return apply(processorScore);
        case ProcessorBack:
          return apply(processorScore).transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
        default:
          {
            System.out.println("Invalid score state");
            return poseManager.getPose();
          }
      }
    };
  }

  public static enum ScoreState {
    LeftBranch,
    RightBranch,
    Dealgify,
    ProcessorFront,
    ProcessorBack
  }

  public static enum IntakeState {
    Source,
    Ice_Cream,
    Ground
  }
}
