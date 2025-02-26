package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.RobotCommands.ScoreState.Dealgify;
import static frc.robot.RobotCommands.dealgify;
import static frc.robot.RobotCommands.goalPose;
import static frc.robot.RobotCommands.scoreCoral;
import static frc.robot.RobotCommands.scoreProcessor;
import static frc.robot.RobotCommands.scoreState;
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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
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
  private static final boolean isChoreoAuto = false;

  private int coralOnL3 = 0;
  private int coralOnL2 = 0;
  private final LoggedTunableNumber delayAfterAlgaeIntake =
      new LoggedTunableNumber("delayAfterAlgaeIntake", 1);

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
    chooser.addRoutine("CenterCDProcessorAlgaeL2L3", this::CenterCDProcessorAlgaeL2L3);
    chooser.addRoutine("GHAlgaeToProcessorL1", this::GHAlgaeToProcessorL1);
    chooser.addRoutine("FL2HGAlgae", this::FL2HGAlgae);
    chooser.addRoutine("IL2HGAlgae", this::IL2HGAlgae);
    chooser.addRoutine("chaos", this::chaos);

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
    AutoTrajectory KLAlgaeToStationHigh = routine.trajectory("KLAlgaeToStationHigh");
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
            // Score coral on L1
            elevator
                .request(L1)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToLKAlgae.getFinalPose().get(),
                        CenterWallToLKAlgae.done()))
                .withName("ScoreCoralOnL1"));
    CenterWallToLKAlgae.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(
                    // Dealgify
                    runOnce(() -> scoreState = Dealgify),
                    dealgify(
                            elevator,
                            carriage,
                            poseManager,
                            () -> CenterWallToLKAlgae.getFinalPose().get(),
                            () -> !CenterWallToLKAlgae.active().getAsBoolean())
                        .asProxy()
                        .deadlineFor(drive.fullAutoDrive(goalPose(poseManager))),
                    // Start next path once algae is held
                    KLAlgaeToStationHigh.cmd())
                .withName("DealgifyThenGoToStationHigh"));

    // Eject algae while driving
    KLAlgaeToStationHigh.atTime("EjectAlgae").onTrue(carriage.scoreProcessor());

    // Drive back from the station to our next scoring location
    // We're intaking coral with a trigger in Robot.java so we don't need to do it here
    KLAlgaeToStationHigh.done()
        .or(KToStationHigh.done())
        .or(LToStationHigh.done())
        .onTrue(
            waitUntil(() -> carriage.coralHeld())
                .andThen(
                    either(
                        StationHighToL.cmd(),
                        StationHighToK.cmd(),
                        () -> (coralOnL2 + coralOnL3) % 2 == 0) // Alternate K and L
                    )
                .withName("0"));

    StationHighToK.active()
        .and(carriage::coralHeld)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToK.getFinalPose().get(),
                        StationHighToK.done())));

    StationHighToK.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(KToStationHigh.cmd()));

    StationHighToL.active()
        .and(carriage::coralHeld)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToL.getFinalPose().get(),
                        StationHighToL.done())));

    StationHighToL.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(LToStationHigh.cmd()));

    return routine;
  }

  public AutoRoutine CenterCDProcessorAlgaeL2L3() {
    AutoRoutine routine = factory.newRoutine("CenterCDProcessorAlgaeL2L3");
    AutoTrajectory CenterWallToLKAlgae = routine.trajectory("CenterProcessorToCDAlgae");
    AutoTrajectory KLAlgaeToStationHigh = routine.trajectory("CDToStationLow");
    AutoTrajectory StationHighToK = routine.trajectory("StationLowToC");
    AutoTrajectory KToStationHigh = routine.trajectory("CToStationLow");
    AutoTrajectory StationHighToL = routine.trajectory("StationLowToD");
    AutoTrajectory LToStationHigh = routine.trajectory("DToStationLow");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            CenterWallToLKAlgae.resetOdometry()
                .andThen(CenterWallToLKAlgae.cmd())
                .withName("ResetOdometryAndStartFirstTrajectory"));
    CenterWallToLKAlgae.active()
        .onTrue(
            // Score coral on L1
            elevator
                .request(L1)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToLKAlgae.getFinalPose().get(),
                        CenterWallToLKAlgae.done()))
                .withName("ScoreCoralOnL1"));
    CenterWallToLKAlgae.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(
                    // Dealgify
                    runOnce(() -> scoreState = Dealgify),
                    // dealgify(elevator, carriage, poseManager)
                    //     .asProxy()
                    //     .deadlineFor(drive.fullAutoDrive(goalPose(poseManager))),
                    // Start next path once algae is held
                    KLAlgaeToStationHigh.cmd())
                .withName("DealgifyThenGoToStationHigh"));

    // Eject algae while driving
    KLAlgaeToStationHigh.atTime("EjectAlgae").onTrue(carriage.scoreProcessor());

    // Drive back from the station to our next scoring location
    // We're intaking coral with a trigger in Robot.java so we don't need to do it here
    KLAlgaeToStationHigh.done()
        .or(KToStationHigh.done())
        .or(LToStationHigh.done())
        .onTrue(
            waitUntil(() -> carriage.coralHeld())
                .andThen(
                    either(
                        StationHighToL.cmd(),
                        StationHighToK.cmd(),
                        () -> (coralOnL2 + coralOnL3) % 2 == 0) // Alternate K and L
                    )
                .withName("0"));

    StationHighToK.active()
        .and(carriage::coralHeld)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToK.getFinalPose().get(),
                        StationHighToK.done())));

    StationHighToK.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(KToStationHigh.cmd()));

    StationHighToL.active()
        .and(carriage::coralHeld)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToL.getFinalPose().get(),
                        StationHighToL.done())));

    StationHighToL.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(LToStationHigh.cmd()));

    return routine;
  }

  public AutoRoutine GHAlgaeToProcessorL1() {

    AutoRoutine routine = factory.newRoutine("L1HGalgae");

    AutoTrajectory centerWallToHG = routine.trajectory("CenterToHGAlgae");
    AutoTrajectory hGToProcessorScore = routine.trajectory("GHToProcessorScore");

    routine.active().onTrue(centerWallToHG.resetOdometry().andThen(centerWallToHG.cmd()));
    centerWallToHG
        .done()
        .onTrue(
            // Score coral on L1
            elevator
                .request(L1)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> centerWallToHG.getFinalPose().get(),
                        centerWallToHG.done()))
                .withName("ScoreCoralOnL1"));
    centerWallToHG
        .done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(
                    // Dealgify
                    runOnce(() -> scoreState = Dealgify),
                    // dealgify(elevator, carriage, poseManager)
                    //     .asProxy()
                    //     .deadlineFor(drive.fullAutoDrive(goalPose(poseManager))),
                    Commands.waitSeconds(delayAfterAlgaeIntake.get()),
                    // Start next path once algae is held
                    hGToProcessorScore.cmd())
                .withName("DealgifyandScore"));
    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, true, hGToProcessorScore.done()));

    return routine;
  }

  public AutoRoutine FL2HGAlgae() {

    AutoRoutine routine = factory.newRoutine("FL2HGAlgae");

    AutoTrajectory CenterWallToI = routine.trajectory("CenterProcessorToF");
    AutoTrajectory IToGHAlgae = routine.trajectory("FToGHAlgae");
    AutoTrajectory hGToProcessorScore = routine.trajectory("GHToProcessorScore");

    routine.active().onTrue(CenterWallToI.resetOdometry().andThen(CenterWallToI.cmd()));
    CenterWallToI.done()
        .onTrue(
            // Score coral on L1
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToI.getFinalPose().get(),
                        CenterWallToI.done()))
                .withName("ScoreCoralOnL2"));

    CenterWallToI.done().onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(IToGHAlgae.cmd()));

    IToGHAlgae.done()
        .onTrue(
            runOnce(() -> scoreState = Dealgify)
                .andThen(
                    // dealgify(elevator, carriage, poseManager)
                    //     .asProxy()
                    //     .deadlineFor(drive.fullAutoDrive(goalPose(poseManager))),
                    Commands.waitSeconds(delayAfterAlgaeIntake.get()), hGToProcessorScore.cmd()));

    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, true, hGToProcessorScore.done()));

    return routine;
  }

  public AutoRoutine IL2HGAlgae() {

    AutoRoutine routine = factory.newRoutine("IL2HGAlgae");

    AutoTrajectory CenterWallToI = routine.trajectory("CenterWallToI");
    AutoTrajectory IToGHAlgae = routine.trajectory("IToGHAlgae");
    AutoTrajectory hGToProcessorScore = routine.trajectory("GHToProcessorScore");

    routine.active().onTrue(CenterWallToI.resetOdometry().andThen(CenterWallToI.cmd()));
    CenterWallToI.done()
        .onTrue(
            // Score coral on L1
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToI.getFinalPose().get(),
                        CenterWallToI.done()))
                .withName("ScoreCoralOnL2"));

    CenterWallToI.done().onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(IToGHAlgae.cmd()));

    IToGHAlgae.done()
        .onTrue(
            runOnce(() -> scoreState = Dealgify)
                .andThen(
                    // dealgify(elevator, carriage, poseManager)
                    //     .asProxy()
                    //     .deadlineFor(drive.fullAutoDrive(goalPose(poseManager))),
                    Commands.waitSeconds(delayAfterAlgaeIntake.get()), hGToProcessorScore.cmd()));

    hGToProcessorScore
        .done()
        .onTrue(scoreProcessor(carriage, intake, poseManager, true, hGToProcessorScore.done()));

    return routine;
  }

  // Do not mess with this :
  public AutoRoutine chaos() {
    AutoRoutine routine = factory.newRoutine("chaos");

    AutoTrajectory chaos = routine.trajectory("chaos");
    routine.active().onTrue(chaos.resetOdometry().andThen(chaos.cmd()));
    return routine;
  }
}
