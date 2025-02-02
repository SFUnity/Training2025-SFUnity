package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.IntakeState.*;
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
import frc.robot.constantsGlobal.FieldConstants.CoralStation;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Put high level commands here */
public final class RobotCommands {
  public static ScoreState scoreState = Dealgify;
  public static boolean dealgifyAfterPlacing = false;

  public static Command scoreCoral(Elevator elevator, Carriage carriage, PoseManager poseManager) {
    return Commands.waitUntil(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < elevatorSafeExtensionDistanceMeters.get())
        .andThen(elevator.enableElevator().andThen(carriage.placeCoral()));
  }

  public static Command dealgify(Elevator elevator, Carriage carriage, PoseManager poseManager) {
    BooleanSupplier highAlgae = () -> poseManager.closestFace().highAlgae;
    return Commands.waitUntil(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < elevatorSafeExtensionDistanceMeters.get())
        .andThen(
            Commands.either(elevator.request(AlgaeHigh), elevator.request(AlgaeLow), highAlgae)
                .andThen(elevator.enableElevator())
                .alongWith(
                    Commands.either(carriage.highDealgify(), carriage.lowDealgify(), highAlgae)))
        .alongWith(
            select(
                Map.of(
                    LeftBranch,
                    new WaitUntilCommand(
                            () ->
                                poseManager.getDistanceTo(goalPose(poseManager).get())
                                    < ElevatorConstants.subsystemExtentionLimit)
                        .andThen(
                            score(elevator, carriage)
                                .alongWith(print("Running elevator and carriage score")))
                        .andThen(
                            dealgifyAfterPlacing
                                ? runOnce(
                                        () -> {
                                          scoreState = Dealgify;
                                          dealgifyAfterPlacing = false;
                                        })
                                    .andThen(
                                        fullScore(drive, elevator, carriage, intake, poseManager))
                                : none()),
                    Dealgify,
                    dealgify(elevator, carriage, poseManager.closestFace().highAlgae),
                    ProcessorFront,
                    carriage.scoreProcessor(),
                    ProcessorBack,
                    intake.poopCmd().until(() -> !intake.algaeHeld())),
                () -> scoreState == RightBranch ? LeftBranch : scoreState))
        .withName("Score/Dealgify");
  }

  public static Supplier<Pose2d> goalPose(PoseManager poseManager) {
    return () -> {
      switch (scoreState) {
        case LeftBranch:
          return apply(poseManager.closestLeftBranch().getPose());
        case RightBranch:
          return apply(poseManager.closestRightBranch().getPose());
        case Dealgify:
          return apply(poseManager.closestFace().getPose());
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
    return select(
        Map.of(
            Source,
                drive
                    .headingDrive(
                        () -> {
                          return poseManager.closestStation().getRotation();
                        })
                    .until(carriage::coralHeld)
                    .asProxy(),
            Ground, intake.intakeCmd().asProxy(),
            Ice_Cream, carriage.lowDealgify().asProxy()),
        () -> intakeState);
  }

  public static enum IntakeState {
    Source,
    Ice_Cream,
    Ground
  }
}
