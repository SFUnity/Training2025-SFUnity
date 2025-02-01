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
import edu.wpi.first.wpilibj2.command.Commands;
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
  public static ScoreState scoreState = Dealgify;
  public static boolean dealgifyAfterPlacing = false;

  public static Command scoreCoral(Elevator elevator, Carriage carriage, PoseManager poseManager) {
    return elevator
        .enableElevator()
        .andThen(carriage.placeCoral())
        .onlyIf(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < elevatorSafeExtensionDistanceMeters.get());
  }

  public static Command dealgify(Elevator elevator, Carriage carriage, PoseManager poseManager) {
    BooleanSupplier highAlgae = () -> poseManager.closestFace().highAlgae;
    return Commands.either(
            elevator.request(AlgaeHigh),
            elevator.request(AlgaeLow),
            highAlgae)
        .andThen(elevator.enableElevator())
        .alongWith(
            Commands.either(
                carriage.highDealgify(),
                carriage.lowDealgify(),
                highAlgae))
        .onlyIf(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < elevatorSafeExtensionDistanceMeters.get());
  }

  public static Command scoreProcessor(
      Carriage carriage, Intake intake, PoseManager poseManager, boolean front) {
    return Commands.either(carriage.scoreProcessor(), intake.poopCmd(), () -> front)
        .onlyIf(
            () ->
                poseManager.getDistanceTo(goalPose(poseManager).get())
                    < processorScoreDistanceMeters.get());
  }

  public static Supplier<Pose2d> goalPose(PoseManager poseManager) {
    return () -> {
      switch (scoreState) {
        case LeftBranch:
          return apply(poseManager.closestFace().leftBranch.getPose());
        case RightBranch:
          return apply(poseManager.closestFace().rightBranch.getPose());
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
    return Commands.select(
        Map.of(
            Source,
                drive
                    .headingDrive(
                        () -> {
                          return poseManager.closestStation().getRotation();
                        })
                    .until(carriage::coralHeld),
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
