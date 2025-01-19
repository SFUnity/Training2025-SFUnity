package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    //.bind("scoreAlgae", ScoreAlgaeCMD())

    chooser = new AutoChooser();
        // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);
      chooser.addRoutine("WallLKAlgaeL2L3",this::WallLKAlgaeL2L3);
      chooser.addRoutine("ProcessorCDAlgaeL2L3",this::ProcessorCDAlgaeL2L3);
      chooser.addRoutine("ProcessorCDAlgaeProcessorL2L3",this::ProcessorCDAlgaeProcessorL2L3);
      chooser.addRoutine("WallJILKAlgaeL2L3",this::WallJILKAlgaeL2L3);
        // Put the auto chooser on the dashboard
       // SmartDashboard.putData(autoChooser);
      SmartDashboard.putData(chooser);
        // Schedule the selected auto during the autonomous period
        //RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
      RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());

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
        // PICK UP CORAL
        factory.trajectoryCmd("StationHighToKLL2") // GO TO CORAL
        // PLACE CORAL
        );
  }

  public Command score() {
    return Commands.sequence(Commands.print("scoring"));
  }

  public Command intake() {
    return Commands.sequence(Commands.print("intake coral"));
  }

  public Command RemoveAlgae() {
    return Commands.sequence(Commands.print("removing Algae"));
  }

  // Options: .done() = when routine is done, AtTime("x") = run cmd on eventMarker,
  // .active().whileTrue() =  Trigger while the trajectory is still running.

  // Triggers can be combined using logical conditions for branching autos:

  public AutoRoutine SpamLsAuto() {

    AutoRoutine routine = factory.newRoutine("CenterWallLKAlgaeL1");

    // Load the routine's trajectories
    AutoTrajectory centerToLK = routine.trajectory("CenterWallToLK");
    AutoTrajectory lKToStationHigh = routine.trajectory("KLtoStationHigh");
    AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToKLL1");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerToLK.cmd().beforeStarting(Commands.print("moving to LK")),
                centerToLK.resetOdometry(),
                centerToLK.cmd() // start traj
                ));

    centerToLK
        .done()
        .onTrue( // WHEN WE FINISH LAST PATH
            Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
                RemoveAlgae()
                    .beforeStarting(
                        Commands.print("removing Algae")), // BEFORE REMOVING THE ALGAE PRINT MSG
                RemoveAlgae(), // RUN REMOVE ALGAE CMD
                score().beforeStarting(Commands.print("scoring")), // BEFORE SCORING CORAL PRINT MSG
                score(), // SCORE CORAL
                lKToStationHigh.cmd() // START NEXT PATH
                ));
    lKToStationHigh
        .done()
        .onTrue(
            Commands.sequence(
                intake(),
                stationHighToLKL2
                    .cmd()
                    .beforeStarting(Commands.print("Arrived at Station High, moving to LK")),
                stationHighToLKL2.cmd()));
    return routine;
}


public AutoRoutine WallLKAlgaeL2L3 () {
  AutoRoutine routine = factory.newRoutine("WallLKAlgaeL2L3");

  AutoTrajectory centerWallToLK = routine.trajectory("CenterWallToLK");
  AutoTrajectory lKToStationHigh = routine.trajectory("LKtoStationHigh");
  AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToLKL2");
  
    
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing WallLKAlgaeL2L3 Auto!"),
          centerWallToLK.resetOdometry(),
          centerWallToLK.cmd() 
      )
  );
  
  centerWallToLK.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae LK, moving to Station High!"),
      lKToStationHigh.cmd()
   )
);

for (int i = 0; i < 3; i++) {
  lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to Algae LK L2!"),
      stationHighToLKL2.cmd()
  )
  );
  stationHighToLKL2.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Algae LK L2, moving to Station High!"),
      lKToStationHigh.cmd()
      )
  );
}

lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to Algae LK L2!"),
      stationHighToLKL2.cmd()
  )
  );
  return routine;
}
public AutoRoutine ProcessorCDAlgaeL2L3 () {
  AutoRoutine routine = factory.newRoutine("ProcessorCDAlgaeL2L3");

  AutoTrajectory processorScoreToCD = routine.trajectory("ProcessorScoreToCD");
  AutoTrajectory cDToStationLow = routine.trajectory("cDToStationLow");
  AutoTrajectory stationLowToCD = routine.trajectory("StationLowToCD");
  
    
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing ProcessorCDAlgaeL2L3 Auto!"),
          processorScoreToCD.resetOdometry(),
          processorScoreToCD.cmd() 
      )
  );
  
  processorScoreToCD.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at CD, moving to Station Low!"),
      cDToStationLow.cmd()
   )
);

for (int i = 0; i < 3; i++) {
  cDToStationLow.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station Low, moving to Algae CD L2!"),
      stationLowToCD.cmd()
  )
  );
  stationLowToCD.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Algae CD L2, moving to Station Low!"),
      cDToStationLow.cmd()
      )
  );
}

cDToStationLow.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station Low, moving to Algae CD L2!"),
      stationLowToCD.cmd()
  )
  );
  return routine;
}
public AutoRoutine ProcessorCDAlgaeProcessorL2L3 () {
  AutoRoutine routine = factory.newRoutine("ProcessorCDAlgaeProcessorL2L3");

  AutoTrajectory processorScoreToCD = routine.trajectory("ProcessorScoreToCD");
  AutoTrajectory cDToStationLow = routine.trajectory("cDToStationLow");
  AutoTrajectory stationLowToCD = routine.trajectory("StationLowToCD");
  AutoTrajectory processorScoreToStationLow = routine.trajectory("ProcessorScoreToStationLow");
  AutoTrajectory cDToprocessorScore = routine.trajectory("CDToprocessorScore");
  
    
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing ProcessorCDAlgaeL2L3 Auto!"),
          processorScoreToCD.resetOdometry(),
          processorScoreToCD.cmd() 
      )
  );
  
  processorScoreToCD.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae CD, moving to Processor Score!"),
      cDToprocessorScore.cmd()
   )
);

  cDToprocessorScore.done().onTrue(
  Commands.sequence(
      Commands.print("Arrived at Processor Score, moving to Station Low!"),
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Algae Scored!"),
      processorScoreToStationLow.cmd()
   )
);

  processorScoreToStationLow.done().onTrue(
  Commands.sequence(
    Commands.print("Arrived at Station Low, moving to Algae CD!"),
    stationLowToCD.cmd()
)
);
for (int i = 0; i < 2; i++) {
  
  stationLowToCD.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Algae CD, moving to Station Low!"),
      cDToStationLow.cmd()
      )
  );

  cDToStationLow.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station Low, moving to Algae CD!"),
      stationLowToCD.cmd()
  )
  );
}
  return routine;
}
public AutoRoutine WallJILKAlgaeL2L3 () {
  AutoRoutine routine = factory.newRoutine("WallJILKAlgaeL2L3");

  AutoTrajectory centerWallToJI = routine.trajectory("CenterWallToJI");
  AutoTrajectory lKToStationHigh = routine.trajectory("LKtoStationHigh");
  AutoTrajectory stationHighToLKL2 = routine.trajectory("StationHighToLKL2");
  AutoTrajectory jIToKLAlgae = routine.trajectory("JIToKLAlgae");
  
  
    
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing WallLKAlgaeL2L3 Auto!"),
          centerWallToJI.resetOdometry(),
          centerWallToJI.cmd() 
      )
  );
  
  centerWallToJI.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae JI, moving to Algae LK!"),
      jIToKLAlgae.cmd()
   )
);

  jIToKLAlgae.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae LK, moving to Station High!"),
      lKToStationHigh.cmd()
   )
);

for (int i = 0; i < 3; i++) {
  lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to Algae LK!"),
      stationHighToLKL2.cmd()
  )
  );
  stationHighToLKL2.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Algae LK, moving to Station High!"),
      lKToStationHigh.cmd()
      )
  );
}

lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to Algae LK!"),
      stationHighToLKL2.cmd()
  )
  );
  return routine;
}  
// Commands
  
public AutoRoutine HGAlgaeL1ScoreEFAlgaeCDAlgaeScore() {

  AutoRoutine routine = factory.newRoutine("HGAlgaeL1ScoreEFAlgaeCDAlgaeScore");

  // Load the routine's trajectories
  AutoTrajectory centerToGH = routine.trajectory("CenterToHGAlgae");
  AutoTrajectory gHToProcessor = routine.trajectory("GHToProcessorScore");
  AutoTrajectory processorToEF = routine.trajectory("ProcessorScoreToEFAlgae");
  AutoTrajectory eFToCDAlgae = routine.trajectory("EFToCDAlgae");
  AutoTrajectory cDToProcessorScore = routine.trajectory("EFToProcessorScore");

  // When the routine begins, reset odometry and start the first trajectory
  routine
      .active()
      .onTrue(
          Commands.sequence(
              centerToLK.resetOdometry(),
              centerToGH.cmd() // start traj
              ));

  centerToGH.done().onTrue( // WHEN WE FINISH LAST PATH
          Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
              score(),
              RemoveAlgae(),
              RemoveAlgae(), // RUN REMOVE ALGAE CMD
              gHToProcessor.cmd() // START NEXT PATH
              ));
  gHToProcessor.done().onTrue(
          Commands.sequence(
              intake(),
              processorToEF.cmd()));
  processorToEF.done().onTrue(
         Commands.sequence(
               //remove Algae
              eFToCDAlgae.cmd()
                ));
  eFToCDAlgae.done().onTrue( // WHEN WE FINISH LAST PATH
           Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
              RemoveAlgae(), // RUN REMOVE ALGAE CMD
              cDToProcessorScore.cmd() // START NEXT PATH
              ));
  return routine;
}
}
