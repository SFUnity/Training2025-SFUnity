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
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Put high level commands here */
public final class RobotCommands {
  public static boolean allowAutoDrive = false;
  public static boolean allowHeadingAlign = false;
  public static ScoreState scoreState = Dealgify;
  public static boolean dealgifyAfterPlacing = false;

  public static Command scoreCoral(
      Elevator elevator, Carriage carriage, PoseManager poseManager, BooleanSupplier atPose) {
    return scoreCoral(elevator, carriage, poseManager, goalPose(poseManager), atPose);
  }

  public static Command scoreCoral(
      Elevator elevator,
      Carriage carriage,
      PoseManager poseManager,
      Supplier<Pose2d> goalPose,
      BooleanSupplier atPose) {
    return waitUntil(
            () ->
                !allowAutoDrive
                    || poseManager.getDistanceTo(goalPose.get())
                        < elevatorSafeExtensionDistanceMeters.get())
        .andThen(
            elevator
                .enableElevator()
                .alongWith(
                    either(
                            waitUntil(elevator::pastL3Height).andThen(carriage.backUpForL3()),
                            none(),
                            () -> elevator.goalHeightInches > ElevatorConstants.pastL3Height.get())
                        .andThen(
                            waitUntil(() -> atPose.getAsBoolean() && elevator.atDesiredHeight()),
                            carriage.placeCoral())));
  }

  public static Command dealgify(Elevator elevator, Carriage carriage, PoseManager poseManager) {
    return dealgify(elevator, carriage, poseManager, goalPose(poseManager));
  }

  public static Command dealgify(
      Elevator elevator, Carriage carriage, PoseManager poseManager, Supplier<Pose2d> goalPose) {
    BooleanSupplier highAlgae = () -> poseManager.closestFaceHighAlgae();
    return waitUntil(
            () ->
                !allowAutoDrive
                    || poseManager.getDistanceTo(goalPose.get())
                        < elevatorSafeExtensionDistanceMeters.get())
        .andThen(
            either(elevator.request(AlgaeHigh), elevator.request(AlgaeLow), highAlgae)
                .andThen(elevator.enableElevator())
                .alongWith(either(carriage.highDealgify(), carriage.lowDealgify(), highAlgae)))
        .alongWith(runOnce(() -> Logger.recordOutput("HighAlgae", highAlgae.getAsBoolean())));
  }

  public static Command scoreProcessor(
      Carriage carriage,
      Intake intake,
      PoseManager poseManager,
      boolean front,
      BooleanSupplier atPose) {
    return waitUntil(atPose)
        .andThen(either(carriage.scoreProcessor(), intake.poopCmd(), () -> front));
  }

  public static BooleanSupplier atGoal(Drive drive) {
    return () -> !allowAutoDrive || (drive.linearAtGoal() && drive.thetaAtGoal());
  }

  public static Supplier<Pose2d> goalPose(PoseManager poseManager) {
    return () -> {
      switch (scoreState) {
        default:
          return apply(poseManager.closest(scoreState));
        case ProcessorFront:
          return apply(processorScore);
        case ProcessorBack:
          return apply(processorScore).transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
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
                either(
                    drive
                        .headingDrive(
                            () -> {
                              return poseManager.closestStation().getRotation();
                            })
                        .until(carriage::coralHeld)
                        .asProxy(),
                    carriage.intakeCoral().asProxy(),
                    () -> allowHeadingAlign),
                Ground,
                intake.intakeCmd().asProxy(),
                Ice_Cream,
                carriage.lowDealgify().asProxy()),
            () -> intakeState)
        .withName("fullIntake");
  }

  public static enum IntakeState {
    Source,
    Ice_Cream,
    Ground
  }
}
