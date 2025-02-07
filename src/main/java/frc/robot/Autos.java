package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.RobotCommands.ScoreState.Dealgify;
import static frc.robot.RobotCommands.dealgify;
import static frc.robot.RobotCommands.dealgifyAfterPlacing;
import static frc.robot.RobotCommands.fullIntake;
import static frc.robot.RobotCommands.goalPose;
import static frc.robot.RobotCommands.scoreCoral;
import static frc.robot.RobotCommands.scoreProcessor;
import static frc.robot.RobotCommands.scoreState;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.AlgaeLow;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L1;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L2;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L3;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    /* Set up main choreo routines */
    chooser = new AutoChooser();
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);
    chooser.addRoutine("WallLKAlgaeL2L3", this::WallLKAlgaeL2L3);
    chooser.addRoutine("WallCDAlgaeProcessorScoreL2L3", this::WallCDAlgaeProcessorScoreL2L3);
    chooser.addRoutine("L1HGAlgae", this::L1HGAlgae);
    chooser.addRoutine("FL2HGAlgae", this::FL2HGAlgae);
    chooser.addRoutine("IL2HGAlgae", this::IL2HGAlgae);

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
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToLKAlgae.getFinalPose().get(),
                        CenterWallToLKAlgae.done()),
                    runOnce(() -> scoreState = Dealgify),
                    dealgify(elevator, carriage, poseManager)
                        .deadlineFor(drive.fullAutoDrive(goalPose(poseManager)))));
    CenterWallToLKAlgae.done()
        .onTrue(
            waitUntil(() -> carriage.algaeHeld())
                .andThen(LKToStationHigh.cmd())
                .withName("waitUntilAlgaeHeldThenLKCmd"));

    LKToStationHigh.atTime("EjectAlgae").onTrue(carriage.scoreProcessor());

    LKToStationHigh.done()
        .or(KToStationHigh.done())
        .or(LToStationHigh.done())
        .onTrue( // may need to add a small wait command here depending on how mechanical works
            either(StationHighToL.cmd(), StationHighToK.cmd(), () -> coralOnL2 >= 1));
    // For intaking coral see robot.configureBindings, state-based triggers, all the time
    StationHighToK.active()
        .and(carriage::coralHeld)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 1)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToK.getFinalPose().get(),
                        StationHighToK.done())));
    StationHighToK.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(KToStationHigh.cmd()));
    // StationHighToL.done().onTrue(getAutonomousCommand());

    return routine;
  }

  public AutoRoutine WallCDAlgaeProcessorScoreL2L3() {

    AutoRoutine routine = factory.newRoutine("WallCDAlgaeProcessorScoreL2L3");

    AutoTrajectory centerWallToCD = routine.trajectory("CenterWallToCD");
    AutoTrajectory cDToProcessorScore = routine.trajectory("CDToProcessorScore");
    AutoTrajectory processorScoreToCD = routine.trajectory("ProcessorScoreToCD");
    AutoTrajectory cDToStationLow = routine.trajectory("CDToStationLow");
    AutoTrajectory stationLowToCD = routine.trajectory("StationLowToCD");

    routine
        .active()
        .onTrue(
            centerWallToCD
                .resetOdometry()
                .andThen(elevator.request(L2), scoreCoral(elevator, carriage, poseManager, null))
                .andThen(cDToProcessorScore.cmd()));
    cDToProcessorScore
        .active()
        .onTrue(
            elevator
                .request(AlgaeLow)
                .andThen(
                    runOnce(() -> scoreState = Dealgify),
                    dealgify(elevator, carriage, poseManager),
                    processorScoreToCD.cmd()));
    processorScoreToCD
        .active()
        .onTrue(
            scoreProcessor(carriage, intake, poseManager, dealgifyAfterPlacing, null)
                .andThen(cDToStationLow.cmd()));

    cDToStationLow
        .active()
        .onTrue(fullIntake(drive, carriage, intake, poseManager).andThen(stationLowToCD.cmd()));

    stationLowToCD
        .active()
        .onTrue(elevator.request(L3).andThen(scoreCoral(elevator, carriage, poseManager, null)));

    return routine;
  }

  public AutoRoutine L1HGAlgae() {

    AutoRoutine routine = factory.newRoutine("L1HGalgae");

    AutoTrajectory centerWallToHG = routine.trajectory("CenterWallToHG");
    AutoTrajectory hGToProcessorScore = routine.trajectory("HGToProcessorScore");

    routine.active().onTrue(centerWallToHG.resetOdometry().andThen(centerWallToHG.cmd()));
    centerWallToHG
        .done()
        .onTrue(
            elevator
                .request(L1)
                .andThen(scoreCoral(elevator, carriage, poseManager, null))
                .andThen(
                    elevator.request(AlgaeLow),
                    runOnce(
                        () -> {
                          scoreState = Dealgify;
                        }),
                    dealgify(elevator, carriage, poseManager))
                .andThen(hGToProcessorScore.cmd()));
    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, dealgifyAfterPlacing, null));

    return routine;
  }

  public AutoRoutine FL2HGAlgae() {

    AutoRoutine routine = factory.newRoutine("FL2HGAlgae");

    AutoTrajectory centerWallToEFAlgae = routine.trajectory("CenterWallToEFAlgae");
    AutoTrajectory eFToGHAlgae = routine.trajectory("EFToGHAlgae");
    AutoTrajectory hGToProcessorScore = routine.trajectory("HGToProcessorScore");

    routine.active().onTrue(centerWallToEFAlgae.resetOdometry().andThen(centerWallToEFAlgae.cmd()));
    centerWallToEFAlgae
        .done()
        .onTrue(
            elevator
                .request(L2)
                .andThen(scoreCoral(elevator, carriage, poseManager, null))
                .andThen(eFToGHAlgae.cmd()));

    eFToGHAlgae
        .done()
        .onTrue(
            elevator
                .request(AlgaeLow)
                .andThen(
                    runOnce(
                        () -> {
                          scoreState = Dealgify;
                        }))
                .andThen(dealgify(elevator, carriage, poseManager)));

    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, dealgifyAfterPlacing, null));

    return routine;
  }

  public AutoRoutine IL2HGAlgae() {

    AutoRoutine routine = factory.newRoutine("IL2HGAlgae");

    AutoTrajectory centerWallToJIAlgae = routine.trajectory("CenterWallToJIAlgae");
    AutoTrajectory jIToGH = routine.trajectory("JIToGH");
    AutoTrajectory hGToProcessorScore = routine.trajectory("HGToProcessorScore");

    routine.active().onTrue(centerWallToJIAlgae.resetOdometry().andThen(centerWallToJIAlgae.cmd()));
    centerWallToJIAlgae
        .done()
        .onTrue(
            elevator
                .request(L2)
                .andThen(scoreCoral(elevator, carriage, poseManager, null))
                .andThen(jIToGH.cmd()));

    jIToGH
        .done()
        .onTrue(
            elevator
                .request(AlgaeLow)
                .andThen(
                    runOnce(
                        () -> {
                          scoreState = Dealgify;
                        }))
                .andThen(dealgify(elevator, carriage, poseManager)));

    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, dealgifyAfterPlacing, null));

    return routine;
  }
}
