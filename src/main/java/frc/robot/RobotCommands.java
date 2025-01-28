package frc.robot;

import static frc.robot.RobotCommands.IntakeState.*;
import static frc.robot.RobotCommands.ScoreState.*;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constantsGlobal.FieldConstants.CoralStation;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Put high level commands here */
public final class RobotCommands {
  public static Command score(Elevator elevator, Carriage carriage) {
    return Commands.sequence(elevator.enableElevator(), carriage.placeCoral());
  }

  public static Command dealgify(Elevator elevator, Carriage carriage, BooleanSupplier high) {
    return Commands.either(elevator.request(AlgaeHigh), elevator.request(AlgaeLow), high)
        .andThen(elevator.enableElevator())
        .alongWith(Commands.either(carriage.highDealgify(), carriage.lowDealgify(), high))
        .withName("dealgify");
  }

  public static ScoreState scoreState = Dealgify;
  public static boolean dealgifyAfterPlacing = false;

  private static double elevatorSafeExtensionDistanceMeters = 1;
  private static double processorScoreDistanceMeters = 0.1;

  public static Command fullScore(
      Drive drive,
      Elevator elevator,
      Carriage carriage,
      Intake intake,
      PoseManager poseManager,
      Trigger scoreTrigger) {
    return new WaitUntilCommand(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < switch (scoreState) {
                      case LeftBranch, RightBranch, Dealgify -> elevatorSafeExtensionDistanceMeters;
                      case ProcessorFront, ProcessorBack -> processorScoreDistanceMeters;
                    })
        .andThen(
            Commands.select(
                Map.of(
                    LeftBranch,
                    score(elevator, carriage)
                        .finallyDo(
                            () -> {
                              if (dealgifyAfterPlacing) {
                                scoreState = Dealgify;
                                dealgifyAfterPlacing = false;
                                CommandScheduler.getInstance()
                                    .schedule(
                                        fullScore(
                                                drive,
                                                elevator,
                                                carriage,
                                                intake,
                                                poseManager,
                                                scoreTrigger)
                                            .onlyWhile(() -> scoreTrigger.getAsBoolean()));
                              }
                            }),
                    Dealgify,
                    dealgify(elevator, carriage, () -> poseManager.closestFace().highAlgae),
                    ProcessorFront,
                    carriage.scoreProcessor(),
                    ProcessorBack,
                    intake.poopCmd()),
                () -> scoreState == RightBranch ? LeftBranch : scoreState))
        .deadlineFor(drive.fullAutoDrive(goalPose(poseManager)))
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

  public static IntakeState intakeState = Source;

  public static Command fullIntake(
      Drive drive, Carriage carriage, Intake intake, PoseManager poseManager) {
    return Commands.select(
        Map.of(
            Source,
                drive
                    .headingDrive(
                        () -> {
                          final Pose2d leftFaceFlipped = apply(CoralStation.leftCenterFace);
                          final Pose2d rightFaceFlipped = apply(CoralStation.rightCenterFace);
                          Pose2d closerStation;

                          if (poseManager.getDistanceTo(leftFaceFlipped)
                              < poseManager.getDistanceTo(rightFaceFlipped)) {
                            closerStation = leftFaceFlipped;
                          } else {
                            closerStation = rightFaceFlipped;
                          }
                          return closerStation.getRotation();
                        })
                    .withDeadline(carriage.intakeCoral()),
            Ground, intake.intakeCmd(),
            Ice_Cream, carriage.lowDealgify()),
        () -> intakeState);
  }

  public static enum IntakeState {
    Source,
    Ice_Cream,
    Ground
  }
}
