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
      chooser.addRoutine("ProcessorCDAlgaeL2L3",this::CenterGHProcessorEFProcessorCDProcessorAlgaeIL2);
      chooser.addRoutine("ProcessorCDAlgaeProcessorL2L3",this::WallJIL2AlgaeL2L1);
      chooser.addRoutine("WallJILKAlgaeL2L3",this::WallJILKAlgaeL2L3);
      chooser.addRoutine("WallJILKAlgaeL2L3",this::WallJIL2AlgaeL2L1);
      chooser.addRoutine("ProcessorCDAlgaeL2L3",this::CenterJIProcessorGHProcessorEFProcessorAlgaeIL2);
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


 // Options: .done() = when routine is done, AtTime("x") = run cmd on eventMarker, 
  //.active().whileTrue() =  Trigger while the trajectory is still running.
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
public AutoRoutine CenterWallL1() {

  //Triggers can be combined using logical conditions for branching autos:

  // Load the routine's trajectories
  AutoRoutine routine = factory.newRoutine("CenterWallLKAlgaeL1");
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
                 // SCORE CORAL L1 KL
                lKToStationHigh.cmd() // START NEXT PATH
                ));
    lKToStationHigh.done().onTrue(
            Commands.sequence(
                // INTAKE CORAL
                stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( 
                Commands.sequence( 
                    // SCORE CORAL L1 KL
                    lKToStationHigh.cmd()
                    ));
     lKToStationHigh.done().onTrue(
                Commands.sequence(
                    //intake("high"),
                    stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( 
                    Commands.sequence(
                        // SCORE CORAL L1 KL
                        lKToStationHigh.cmd() 
                        ));
      lKToStationHigh.done().onTrue(
                    Commands.sequence(
                        // INTAKE CORAL
                        stationHighToLKL2.cmd()));       
    return routine;
}
public AutoRoutine WallLKAlgaeL2L3 () {
  AutoRoutine routine = factory.newRoutine("CenterWallLKAlgaeL1");
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
                 // SCORE CORAL L1 KL
                 // REMOVE ALGAE L2 KL
                lKToStationHigh.cmd() // START NEXT PATH
                ));
    lKToStationHigh.done().onTrue(
            Commands.sequence(
                // INTAKE CORAL
                stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( 
                Commands.sequence( 
                    // SCORE CORAL L2/3 KL
                    lKToStationHigh.cmd()
                    ));
     lKToStationHigh.done().onTrue(
                Commands.sequence(
                    //intake("high"),
                    stationHighToLKL2.cmd()));
     stationHighToLKL2.done().onTrue( 
                    Commands.sequence(
                        // SCORE CORAL L2/3 KL
                        lKToStationHigh.cmd() 
                        ));
      lKToStationHigh.done().onTrue(
                    Commands.sequence(
                        // INTAKE CORAL
                        stationHighToLKL2.cmd()));       
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
       //SCORE CORAL L2 IJ
       jIToKLAlgae.cmd()
   )
  );
  jIToKLAlgae.done().onTrue(
    Commands.sequence(
       //REMOVE ALGAE L2 LK
       lKToStationHigh.cmd()
   )
  );
  lKToStationHigh.done().onTrue(
    Commands.sequence(
       //INTAKE CORAL
        stationHighToLKL2.cmd()
      )
  );
  stationHighToLKL2.done().onTrue(
      Commands.sequence(
        //SCORE CORAL L2/3
        lKToStationHigh.cmd()
    )
  );
  lKToStationHigh.done().onTrue(
    Commands.sequence(
        //INTAKE CORAL
        stationHighToLKL2.cmd()
      )
  );

  stationHighToLKL2.done().onTrue(
    Commands.sequence(
        //SCORE CORAL L2/3
        lKToStationHigh.cmd()
    ) 
  );
  lKToStationHigh.done().onTrue(
    Commands.sequence(
       //INTAKE CORAL
       stationHighToLKL2.cmd()
      )
  );
  stationHighToLKL2.done().onTrue(
      Commands.sequence(
        //SCORE CORAL L2/3
         lKToStationHigh.cmd()
    )
  );
  lKToStationHigh.done().onTrue(
    Commands.sequence(
        //INTAKE CORAL
        stationHighToLKL2.cmd()
        )
    );
  return routine;

}  
public AutoRoutine CenterGHProcessorEFProcessorCDProcessorAlgaeIL2 () {
  AutoRoutine routine = factory.newRoutine("CenterJIProcessorHGProcessorEFProcessorAlgaeIL2");

  AutoTrajectory centerToGH = routine.trajectory("CenterToHGAlgae");
  AutoTrajectory cDprocessorScore = routine.trajectory("CDToProcessorScore");
  AutoTrajectory gHToProcessorScore = routine.trajectory("GHToProcessorScore");
  AutoTrajectory eFToCD = routine.trajectory("EFToCDAlgae");
  AutoTrajectory eFToProcessorScore = routine.trajectory("eFToProcessorScore");
  AutoTrajectory processorScoreToEFAlgae = routine.trajectory("ProcessorScoreToEFAlgae");
  AutoTrajectory processorScoreToCDAlgae = routine.trajectory("ProcessorScoreToCD");
  
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing CenterGHProcessorEFProcessorCDProcessorAlgaeIL2 Auto!"),
          centerToGH.resetOdometry(),
          centerToGH.cmd() 
      )
  );
  centerToGH.done().onTrue(
      Commands.sequence(
        //SCORE CORAL L1 GH
        //REMOVE ALGAE L2 GH KEEP
        gHToProcessorScore.cmd() 
    )
  );
  gHToProcessorScore.done().onTrue(
  Commands.sequence(
      //SCORE ALGAE
      processorScoreToEFAlgae.cmd()
   )
  );
  processorScoreToEFAlgae.done().onTrue(
  Commands.sequence(
      //REMOVE ALGAE L3 EF 
      eFToCD.cmd()
   )
  );
  eFToCD.done().onTrue(
  Commands.sequence(
        //REMOVE ALGAE L2 CD KEEP
      cDprocessorScore.cmd()
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
              centerToJI.resetOdometry(),
              centerToJI.cmd() // start traj
              ));

      centerToJI.done().onTrue( 
          Commands.sequence(
              //SCORE CORAL L1 JI
              // REMOVE ALGAE L3 JI
              JIToStationHigh.cmd() 
              ));
      JIToStationHigh.done().onTrue(
          Commands.sequence(
            //INTAKE CORAL
            StationHighToJI.cmd()
            ));
      StationHighToJI.done().onTrue(
         Commands.sequence(
            //SCORE CORAL L2/3 JI
            JIToStationHigh.cmd()
            ));
      JIToStationHigh.done().onTrue(
          Commands.sequence(
              //INTAKE CORAL
              StationHighToJI.cmd()
              ));
       StationHighToJI.done().onTrue(
          Commands.sequence(
              //SCORE JI
              JIToStationHigh.cmd()
              ));
  return routine;
}
public AutoRoutine CenterJIProcessorGHProcessorEFProcessorAlgaeIL2 () {
  AutoRoutine routine = factory.newRoutine("CenterJIProcessorHGProcessorEFProcessorAlgaeIL2");

  AutoTrajectory centerToJI = routine.trajectory("CenterToJIAlgae");
  AutoTrajectory cDprocessorScore = routine.trajectory("CDToProcessorScore");
  AutoTrajectory gHToProcessorScore = routine.trajectory("GHToProcessorScore");
  AutoTrajectory eFToCD = routine.trajectory("EFToCDAlgae");
  AutoTrajectory jIToGH = routine.trajectory("jIToGH");
  AutoTrajectory eFToProcessorScore = routine.trajectory("eFToProcessorScore");
  AutoTrajectory processorScoreToEFAlgae = routine.trajectory("ProcessorScoreToEFAlgae");
  AutoTrajectory processorScoreToCDAlgae = routine.trajectory("ProcessorScoreToCD");
  
    routine.active().onTrue(
      Commands.sequence(
          Commands.print("Performing CenterGHProcessorEFProcessorCDProcessorAlgaeIL2 Auto!"),
          centerToJI.resetOdometry(),
          centerToJI.cmd() 
      )
  );
  centerToJI.done().onTrue(
      Commands.sequence(
        //SCORE CORAL L1 GH
        //REMOVE ALGAE L2 GH KEEP
        jIToGH.cmd() 
    )
  );
  jIToGH.done().onTrue(
  Commands.sequence(
      //SCORE ALGAE
      gHToProcessorScore.cmd()
   )
  );
  gHToProcessorScore.done().onTrue(
  Commands.sequence(
      //REMOVE ALGAE L3 EF 
      processorScoreToEFAlgae.cmd()
   )
  );
  return routine;

}

}
