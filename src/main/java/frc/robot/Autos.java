package frc.robot;

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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final AutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public Autos(Drive drive, PoseManager poseManager) {
    this.drive = drive;
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


    //TODO:
    //factory
    //.bind("scoreCoral", SCORECMD())
    //.bind("RemoveAlgae", RemoveAlageCMD())

    chooser = new AutoChooser();
        // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

        // Put the auto chooser on the dashboard
       // SmartDashboard.putData(autoChooser);

        // Schedule the selected auto during the autonomous period
        //RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
        nonChoreoChooser.addOption("Module Drive Tuning", drive.tuneModuleDrive());
        nonChoreoChooser.addOption("Module Turn Tuning", drive.tuneModuleTurn());

        // Set up SysId routines
        nonChoreoChooser.addOption(
            "Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
        nonChoreoChooser.addOption(
            "Drive Simple FF Characterization", drive.feedforwardCharacterization());
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? chooser.selectedCommandScheduler() : nonChoreoChooser.get();
  }

  // Routines
public Command CenterWallLKAlgaeL1() {
    return Commands.sequence(
        factory.resetOdometry("CenterWallToLK"), //   
        // PICK UP ALGAE AND SCORE CORAL
        factory.trajectoryCmd("KLEject"), // GET RID OF ALGAE
        factory.trajectoryCmd("KLtoStationHigh"), // GO TO CORAL PICKUP
        //PICK UP CORAL
         factory.trajectoryCmd("StationHighToKLL2") // GO TO CORAL
         //PLACE CORAL
    );
}
public AutoRoutine SpamLsAuto() {

  // Options: .done() = when routine is done, AtTime("x") = run cmd on eventMarker, 
  //.active().whileTrue() =  Trigger while the trajectory is still running.

  //Triggers can be combined using logical conditions for branching autos:

  AutoRoutine routine = factory.newRoutine("CenterWallLKAlgaeL1");

  // Load the routine's trajectories
  AutoTrajectory centerToLK = routine.trajectory("CenterWallToLK");
  AutoTrajectory lKToStationHigh = routine.trajectory("KLtoStationHigh");
  AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToKLL1");

    // When the routine begins, reset odometry and start the first trajectory 
      routine.active().onTrue(
        Commands.sequence(
            Commands.print("Started CenterWallLKAlgaeL1 Auto!"),
            centerToLK.resetOdometry(),
            centerToLK.cmd()   // start traj
        )
    );

    centerToLK.done().onTrue(
    Commands.sequence(
        Commands.print("Arrived at LK, moving to Station High"),
        lKToStationHigh.cmd()
     )
);
    lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to LK L2"),
      stationHighToLKL2.cmd()
  )
);
    return routine;
}

  // Commands
}
