package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.*;
import static frc.robot.RobotCommands.ScoreState.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final Carriage carriage;
  private final Elevator elevator;
  private final Intake intake;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final AutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  private int coralOnL3 = 0;
  private int coralOnL2 = 0;

  public Autos(
      Drive drive, Carriage carriage, Elevator elevator, Intake intake, PoseManager poseManager) {
    this.drive = drive;
    this.carriage = carriage;
    this.elevator = elevator;
    this.intake = intake;
    this.poseManager = poseManager;

    factory =
        new AutoFactory(
            poseManager::getPose,
            poseManager::setPose,
            drive::followTrajectory,
            true,
            drive,
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              Logger.recordOutput(
                  "Drive/Choreo/Active Traj",
                  (AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj).getPoses());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj End Pose",
                  traj.getFinalPose(AllianceFlipUtil.shouldFlip()).get());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj Start Pose",
                  traj.getInitialPose(AllianceFlipUtil.shouldFlip()).get());
            });

    // TODO:
    // factory
    // .bind("scoreCoral", SCORECMD())
    // .bind("RemoveAlgae", RemoveAlageCMD())

    // .bind("scoreAlgae", ScoreAlgaeCMD())

    /* Set up main choreo routines */
    chooser = new AutoChooser();
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);
    chooser.addRoutine("WallLKAlgaeL2L3", this::WallLKAlgaeL2L3);
    chooser.addRoutine(
        "CenterGHProcessorEFProcessorCDProcessorAlgaeIL2",
        this::CenterGHProcessorEFProcessorCDProcessorAlgaeIL2);
    chooser.addRoutine(
        "CenterJIProcessorGHProcessorEFProcessorAlgaeIL2",
        this::CenterJIProcessorGHProcessorEFProcessorAlgaeIL2);
    chooser.addRoutine("WallJILKAlgaeL2L3", this::WallJILKAlgaeL2L3);
    chooser.addRoutine("WallJIL2AlgaeL2L1", this::WallJIL2AlgaeL2L1);
    chooser.addRoutine("CenterWallL1", this::CenterWallL1);

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
        nonChoreoChooser.addOption("Module Turn Tuning", drive.tuneModuleTurn());
        nonChoreoChooser.addOption("Module Drive Tuning", drive.tuneModuleDrive());

        // Set up SysId routines
        nonChoreoChooser.addOption(
            "Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
        nonChoreoChooser.addOption(
            "Drive Simple FF Characterization", drive.feedforwardCharacterization());
      }
    }

    SmartDashboard.putData("Choreo Chooser", chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? chooser.selectedCommandScheduler() : nonChoreoChooser.get();
  }

  // Options: .done() = when routine is done, AtTime("x") = run cmd on eventMarker,
  // .active().whileTrue() =  Trigger while the trajectory is still running.
  // Routines

  public AutoRoutine CenterWallL1() {

    // Triggers can be combined using logical conditions for branching autos:

    // Load the routine's trajectories
    AutoRoutine routine = factory.newRoutine("CenterWallLKAlgaeL1");
    AutoTrajectory CenterWallToLK = routine.trajectory("CenterWallToLK");
    AutoTrajectory lKToStationHigh = routine.trajectory("KLEjectToStationHigh");
    AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToKLL2");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                CenterWallToLK.resetOdometry(), CenterWallToLK.cmd() // start traj
                ));

    CenterWallToLK.done()
        .onTrue( // WHEN WE FINISH LAST PATH
            Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
                elevator
                    .request(L2)
                    .andThen(score(elevator, carriage))
                    .andThen(
                        runOnce(
                            () -> {
                              scoreState = Dealgify;
                            }),
                        fullScore(drive, elevator, carriage, intake, poseManager)), // Dealgify
                lKToStationHigh.cmd() // START NEXT PATH
                ));
    lKToStationHigh
        .done()
        .onTrue(
            Commands.sequence(
                // INTAKE CORAL
                stationHighToLKL2.cmd()));
    stationHighToLKL2
        .done()
        .onTrue(
            Commands.sequence(
                // SCORE CORAL L1 KL
                lKToStationHigh.cmd()));

    return routine;
  }

  public AutoRoutine WallLKAlgaeL2L3() {
    AutoRoutine routine = factory.newRoutine("WallLKAlgaeL2L3");
    AutoTrajectory CenterWallToLKAlgae = routine.trajectory("CenterWallToLKAlgae");
    AutoTrajectory LKToStationHigh = routine.trajectory("LKAlgaeToStationHigh");
    AutoTrajectory StationHighToK = routine.trajectory("StationHighToK");
    AutoTrajectory KToStationHigh = routine.trajectory("KToStationHigh");
    AutoTrajectory StationHighToL = routine.trajectory("StationHighToL");
    AutoTrajectory LToStationHigh = routine.trajectory("LToStationHigh");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            CenterWallToLKAlgae.resetOdometry()
                .andThen(CenterWallToLKAlgae.cmd())
                .withName("ResetOdometryAndStartFirstTrajectory"));
    CenterWallToLKAlgae.active()
        .onTrue(
            elevator
                .request(L1)
                .andThen(
                    runOnce(() -> scoreState = Dealgify),
                    scoreCoral(elevator, carriage, poseManager),
                    dealgify(elevator, carriage, poseManager)));
    CenterWallToLKAlgae.done()
        .onTrue(
            waitUntil(() -> carriage.algaeHeld())
                .andThen(LKToStationHigh.cmd())
                .withName("waitUntilAlgaeHeldThenLKCmd"));
    // TODO Add binding in choreo to shoot out the algae
    LKToStationHigh.done()
        .or(KToStationHigh.done())
        .or(LToStationHigh.done())
        .onTrue( // may need to add a small wait command here depending on how mechanical works
            either(StationHighToL.cmd(), StationHighToK.cmd(), () -> coralOnL2 >= 1));
    // For intaking coral see robot.configureBindings, state-based triggers, all the time
    StationHighToK.done()
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 1)
                .andThen(scoreCoral(elevator, carriage, poseManager)));
    StationHighToK.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(KToStationHigh.cmd()));
    // StationHighToL.done().onTrue(getAutonomousCommand());

    return routine;
  }

  public AutoRoutine WallJILKAlgaeL2L3() {

    AutoRoutine routine = factory.newRoutine("WallJILKAlgaeL2L3");

    AutoTrajectory centerWallToJI = routine.trajectory("CenterWallToJIAlgae");
    AutoTrajectory lKToStationHigh = routine.trajectory("KLEjectToStationHigh");
    AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToKLL2");
    AutoTrajectory jIToKLAlgae = routine.trajectory("JIToKLAlgae");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerWallToJI.resetOdometry(), centerWallToJI.cmd() // start traj
                ));

    centerWallToJI
        .done()
        .onTrue(
            Commands.sequence(
                elevator
                    .request(L2)
                    .andThen(score(elevator, carriage))
                    .andThen(
                        runOnce(
                            () -> {
                              scoreState = Dealgify;
                            }),
                        fullScore(drive, elevator, carriage, intake, poseManager)), // Dealgify
                jIToKLAlgae.cmd()));
    jIToKLAlgae
        .done()
        .onTrue(
            Commands.sequence(
                runOnce(
                    () -> {
                      scoreState = Dealgify;
                    }),
                fullScore(drive, elevator, carriage, intake, poseManager),
                lKToStationHigh.cmd()));
    lKToStationHigh
        .done()
        .onTrue(
            Commands.sequence(
                // INTAKE CORAL
                stationHighToLKL2.cmd()));
    stationHighToLKL2
        .done()
        .onTrue(
            Commands.sequence(
                elevator.request(L3).andThen(score(elevator, carriage)), lKToStationHigh.cmd()));
    return routine;
  }

  public AutoRoutine CenterGHProcessorEFProcessorCDProcessorAlgaeIL2() {
    AutoRoutine routine = factory.newRoutine("CenterJIProcessorHGProcessorEFProcessorAlgaeIL2");

    AutoTrajectory centerToGH = routine.trajectory("CenterToHGAlgae");
    AutoTrajectory cDprocessorScore = routine.trajectory("CDToProcessorScore");
    AutoTrajectory gHToProcessorScore = routine.trajectory("GHToProcessorScore");
    AutoTrajectory eFToCD = routine.trajectory("EFToCDAlgae");
    AutoTrajectory processorScoreToEFAlgae = routine.trajectory("ProcessorScoreToEFAlgae");

    routine.active().onTrue(Commands.sequence(centerToGH.resetOdometry(), centerToGH.cmd()));
    centerToGH
        .done()
        .onTrue(
            Commands.sequence(
                elevator
                    .request(L2)
                    .andThen(score(elevator, carriage))
                    .andThen(
                        runOnce(
                            () -> {
                              scoreState = Dealgify;
                            }),
                        fullScore(drive, elevator, carriage, intake, poseManager)), // Dealgify
                gHToProcessorScore.cmd()));
    gHToProcessorScore
        .done()
        .onTrue(
            Commands.sequence(
                // score algae
                ));
    processorScoreToEFAlgae
        .done()
        .onTrue(
            Commands.sequence(
                runOnce(
                    () -> {
                      scoreState = Dealgify;
                    }),
                fullScore(drive, elevator, carriage, intake, poseManager),
                eFToCD.cmd()));
    eFToCD
        .done()
        .onTrue(
            Commands.sequence(
                runOnce(
                    () -> {
                      scoreState = Dealgify;
                    }),
                fullScore(drive, elevator, carriage, intake, poseManager),
                cDprocessorScore.cmd()));

    return routine;
  }

  public AutoRoutine WallJIL2AlgaeL2L1() {

    AutoRoutine routine = factory.newRoutine("WallJIL2AlgaeL2L1");

    // Load the routine's trajectories
    AutoTrajectory centerToJI = routine.trajectory("CenterWallToJIAlgae");
    AutoTrajectory JIToStationHigh = routine.trajectory("JIToStationHigh");
    AutoTrajectory StationHighToJI = routine.trajectory("StationHighToJI");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerToJI.resetOdometry(), centerToJI.cmd() // start traj
                ));
    centerToJI
        .done()
        .onTrue(
            Commands.sequence(
                drive.fullAutoDrive(
                    () ->
                        AllianceFlipUtil.apply(
                            poseManager.closestFace().rightBranch.getPose())), // get closest branch
                // elevator
                //     .request(L1)
                //     .andThen(RobotCommands.score(elevator, carriage)), // score on L1
                // //Tell next pos (L3)
                drive.fullAutoDrive(() -> poseManager.closestFace().getPose()),
                elevator
                    .request(AlgaeHigh)
                    .andThen(elevator.enableElevator()) // delagify pt1
                    .andThen(carriage.highDealgify())
                    .andThen(elevator.disableElevator()), // delgify pt2
                JIToStationHigh.cmd()));
    JIToStationHigh.done()
        .onTrue(
            Commands.sequence(
                carriage.intakeCoral().until(carriage::coralHeld),
                StationHighToJI.cmd().alongWith(elevator.request(L3))));
    StationHighToJI.done()
        .onTrue(
            Commands.sequence(
                drive.fullAutoDrive(
                    () ->
                        AllianceFlipUtil.apply(
                            poseManager.closestFace().rightBranch.getPose())), // get closest branch
                // elevator
                //     .request(L3)
                //     .andThen(RobotCommands.score(elevator, carriage)), // score on L3
                JIToStationHigh.cmd()));
    return routine;
  }

  public AutoRoutine CenterJIProcessorGHProcessorEFProcessorAlgaeIL2() {

    AutoRoutine routine = factory.newRoutine("CenterJIProcessorHGProcessorEFProcessorAlgaeIL2");

    AutoTrajectory centerToJI = routine.trajectory("CenterWallToJIAlgae");
    AutoTrajectory gHToProcessorScore = routine.trajectory("GHToProcessorScore");
    AutoTrajectory jIToGH = routine.trajectory("JIToGH");
    AutoTrajectory eFToProcessorScore = routine.trajectory("eFToProcessorScore");
    AutoTrajectory processorScoreToEFAlgae = routine.trajectory("ProcessorScoreToEFAlgae");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("Performing Auto!"),
                centerToJI.resetOdometry(),
                centerToJI.cmd(),
                Commands.print("xcvbn")));
    centerToJI
        .done()
        .onTrue(
            Commands.sequence(
                elevator
                    .request(L2)
                    .andThen(score(elevator, carriage))
                    .andThen(
                        runOnce(
                            () -> {
                              scoreState = Dealgify;
                            }),
                        fullScore(drive, elevator, carriage, intake, poseManager)),
                jIToGH.cmd()));
    jIToGH
        .done()
        .onTrue(
            Commands.sequence(
                // SCORE ALGAE
                gHToProcessorScore.cmd()));
    gHToProcessorScore
        .done()
        .onTrue(
            Commands.sequence(
                elevator.request(L3).andThen(RobotCommands.score(elevator, carriage)),
                processorScoreToEFAlgae.cmd()));
    processorScoreToEFAlgae.done().onTrue(Commands.sequence(eFToProcessorScore.cmd()));
    return routine;
  }
}
