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
    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerToLK.cmd().beforeStarting(Commands.print("moving to LK")),
                centerToLK.resetOdometry(),
                centerToLK.cmd() // start traj
                ));

    centerToLK.done().onTrue( // WHEN WE FINISH LAST PATH
            Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
                score("lk"), // SCORE CORAL
                 RemoveAlgae("lk"), // RUN REMOVE ALGAE CMD
                lKToStationHigh.cmd() // START NEXT PATH
                ));
    lKToStationHigh.done().onTrue(
            Commands.sequence(
                intake("high"),
                stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( // WHEN WE FINISH LAST PATH
                Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
                    score("lk"), // SCORE CORAL
                    lKToStationHigh.cmd() // START NEXT PATH
                    ));
     lKToStationHigh.done().onTrue(
                Commands.sequence(
                    intake("high"),
                    stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( // WHEN WE FINISH LAST PATH
                    Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
                        score("lk"), // SCORE CORAL
                        lKToStationHigh.cmd() // START NEXT PATH
                        ));
      lKToStationHigh.done().onTrue(
                    Commands.sequence(
                        intake("high"),
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
  stationHighToLKL2.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Algae LK L2, moving to Station High!"),
      lKToStationHigh.cmd()
      )
  );
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

lKToStationHigh.done().onTrue(
    Commands.sequence(
      Commands.print("Arrived at Station High, moving to Algae LK L2!"),
      stationHighToLKL2.cmd()
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
public AutoRoutine CenterJIProcessorHGProcessorEFProcessorAlgaeIL2 () {
  AutoRoutine routine = factory.newRoutine("CenterJIProcessorHGProcessorEFProcessorAlgaeIL2");

  AutoTrajectory centerWallToJI = routine.trajectory("CenterWallToJI");
  AutoTrajectory jIToProcessorScore = routine.trajectory("JIToProcessorScore");
  AutoTrajectory gHToProcessorScore = routine.trajectory("GHToProcessorScore");
  AutoTrajectory eFToProcessorScore = routine.trajectory("EFToProcessorScore");
  AutoTrajectory processorScoreToEFAlgae = routine.trajectory("ProcessorScoreToEFAlgae");
  AutoTrajectory processorScoreToHGAlgae = routine.trajectory("ProcessorScoreToHGAlgae");
  
    
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing CenterJIProcessorHGProcessorEFProcessorAlgaeIL2 Auto!"),
          centerWallToJI.resetOdometry(),
          centerWallToJI.cmd() 
      )
  );
  
    centerWallToJI.done().onTrue(
      Commands.sequence(
        //Placeholder; no meaning or use until Subsystem team gets their stuff done
        Commands.print("Dealgaefied!"),
        Commands.print("Arrived at Algae JI, moving to Processor Score!"),
        jIToProcessorScore.cmd() 
    )
);

  jIToProcessorScore.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
    Commands.print("Algae Scored!"),
      Commands.print("Arrived at Processor Score, moving to Algae EF!"),
      processorScoreToEFAlgae.cmd()
   )
);
processorScoreToEFAlgae.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae EF, moving to Processor Score!"),
      eFToProcessorScore.cmd()
   )
);

  eFToProcessorScore.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Algae Scored!"),
      Commands.print("Arrived at Processor Score, moving to Algae HG!"),
      processorScoreToHGAlgae.cmd()
   )
);

processorScoreToHGAlgae.done().onTrue(
  Commands.sequence(
      //Placeholder; no meaning or use until Subsystem team gets their stuff done
      Commands.print("Dealgaefied!"),
      Commands.print("Arrived at Algae HG, moving to Processor Score!"),
      gHToProcessorScore.cmd()
   )
);


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
              centerToLK.resetOdometry(),
              centerToJI.cmd() // start traj
              ));

              centerToJI.done().onTrue( // WHEN WE FINISH LAST PATH
          Commands.sequence( // RUN THESE COMMANDS IN SEQUENTIAL ORDER
              score("ji"),
              RemoveAlgae("ji"), // RUN REMOVE ALGAE CMD
              JIToStationHigh.cmd() // START NEXT PATH
              ));
      JIToStationHigh.done().onTrue(
          Commands.sequence(
            StationHighToJI.cmd()));

      StationHighToJI.done().onTrue(
         Commands.sequence(
               score("ji")
               JIToStationHigh.cmd()
                ));
                JIToStationHigh.done().onTrue(
                  Commands.sequence(
                    StationHighToJI.cmd()));
        
              StationHighToJI.done().onTrue(
                 Commands.sequence(
                       score("ji")
                       JIToStationHigh.cmd()
                        ));
  return routine;
}


