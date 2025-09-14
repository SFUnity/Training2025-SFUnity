// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.*;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.leftName;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.rightName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constantsGlobal.BuildConstants;
import frc.robot.constantsGlobal.Constants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMixed;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.WPILOGWriter9038;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Autos
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  // Alerts
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 12.5;
  private static final double extraLowBatteryVoltage = 11.5;
  private static final double lowBatteryDisabledTime = 1.5;

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);
  private final Alert noLoggingAlert =
      new Alert("AdvantageKit Failed to open output log file.", AlertType.kError);

  // Subsystems
  private final Drive drive;
  private final AprilTagVision vision;

  // Non-subsystems
  private final PoseManager poseManager = new PoseManager();
  private final Autos autos;

  // Controllers + driving
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  public boolean slowMode = true;
  private final LoggedTunableNumber slowDriveMultiplier =
      new LoggedTunableNumber("Slow Drive Multiplier", 1);
  private final LoggedTunableNumber slowTurnMultiplier =
      new LoggedTunableNumber("Slow Turn Multiplier", 0.7);

  private final DriveCommandsConfig driveCommandsConfig =
      new DriveCommandsConfig(driver, () -> slowMode, slowDriveMultiplier, slowTurnMultiplier);

  private BooleanSupplier loggingOutputAvailable = () -> true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  public Robot() {
    super();

    Leds.getInstance();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        WPILOGWriter9038 logWriter = new WPILOGWriter9038();
        // loggingOutputAvailable = logWriter::hasLogOutput;
        Logger.addDataReceiver(logWriter);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.registerURCL(URCL.startExternal()); // Enables REV CAN logging !!! not replayable !!!
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter()); // for sim logging
        break;

      case REPLAY:
        // In this case when you "Simulate Robot Code" you will enter replay mode
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.

    // Start AdvantageKit logger
    Logger.start();

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    disabledTimer.restart();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMixed(0),
                new ModuleIOMixed(1),
                new ModuleIOMixed(2),
                new ModuleIOMixed(3),
                // new GyroIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                poseManager,
                driveCommandsConfig);
        vision =
            new AprilTagVision(
                poseManager,
                new AprilTagVisionIOLimelight(leftName),
                new AprilTagVisionIOLimelight(rightName));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseManager,
                driveCommandsConfig);
        vision =
            new AprilTagVision(poseManager, new AprilTagVisionIO() {}, new AprilTagVisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                poseManager,
                driveCommandsConfig);
        vision =
            new AprilTagVision(
                poseManager,
                new AprilTagVisionIO() {
                  @Override
                  public String getName() {
                    return leftName;
                  }
                },
                new AprilTagVisionIO() {
                  @Override
                  public String getName() {
                    return rightName;
                  }
                });
        break;
    }

    autos = new Autos(drive, poseManager);

    // Configure the button bindings
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.kInfo).set(true);
    }

    DriverStation.silenceJoystickConnectionWarning(true);

    // For tuning visualizations
    // Logger.recordOutput("ZeroedPose2d", new Pose2d());
    // Logger.recordOutput("ZeroedPose3d", new Pose3d[] {new Pose3d(), new Pose3d()});

    // Set up port forwarding for limelights so we can connect to them through the RoboRIO USB port
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, leftName + ".local", port);
      PortForwarder.add(port + 10, rightName + ".local", port);
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    // Check if logging output is available
    noLoggingAlert.set(!loggingOutputAvailable.getAsBoolean());
    if (!loggingOutputAvailable.getAsBoolean()) {
      DriverStation.reportError("[AdvantageKit] Failed to open output log file.", false);
    }

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
        Leds.getInstance().autoFinished = true;
        Leds.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
      }
    }

    // Check controllers
    driverDisconnected.set(!isControllerConnected(driver));
    operatorDisconnected.set(!isControllerConnected(operator));
    Logger.recordOutput("Controls/driverConnected", isControllerConnected(driver));
    Logger.recordOutput("Controls/operatorConnected", isControllerConnected(operator));

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      double voltage = RobotController.getBatteryVoltage();
      if (voltage <= extraLowBatteryVoltage) {
        lowBatteryAlert.set(true);
        Leds.getInstance().extraLowBatteryAlert = true;
      } else if (voltage <= lowBatteryVoltage) {
        lowBatteryAlert.set(true);
        Leds.getInstance().lowBatteryAlert = true;
      }
    }

    // Logs
    Logger.recordOutput("Controls/intakeState", intakeState.toString());
    Logger.recordOutput("Controls/scoreState", scoreState.toString());
    Logger.recordOutput("Controls/dealgifyAfterPlacing", dealgifyAfterPlacing);
    Logger.recordOutput("Controls/allowAutoDrive", allowAutoDrive);
    Logger.recordOutput("Controls/goalPose", goalPose(poseManager).get());
  }

  private boolean isControllerConnected(CommandXboxController controller) {
    return controller.isConnected()
        && DriverStation.getJoystickIsXbox(
            controller.getHID().getPort()); // Should be an XBox controller
  }

  // Consider moving to its own file if/when it gets big
  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    boolean testDrive = false;

    // Default cmds
    if (testDrive) {
      drive.setDefaultCommand(
          drive.moduleTestingCommand(
              driveCommandsConfig::getXInput, driveCommandsConfig::getOmegaInput));
    } else {
      drive.setDefaultCommand(drive.joystickDrive());
    }

    // Driver controls
    driver.rightTrigger().onTrue(runOnce(() -> Drive.nitro = !Drive.nitro));
    if (testDrive) {
      driver.y().onTrue(drive.setModuleToTest(0));
      driver.x().onTrue(drive.setModuleToTest(1));
      driver.a().onTrue(drive.setModuleToTest(2));
      driver.b().onTrue(drive.setModuleToTest(3));
      driver.start().onTrue(runOnce(() -> drive.allModules = !drive.allModules));
    } else {
      driver
          .y()
          .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(0)).until(drive::thetaAtGoal));
      driver
          .x()
          .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(90)).until(drive::thetaAtGoal));
      driver
          .a()
          .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(180)).until(drive::thetaAtGoal));
      driver
          .b()
          .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(270)).until(drive::thetaAtGoal));
      driver
          .start()
          .onTrue(
              runOnce(
                      () ->
                          poseManager.setPose(
                              new Pose2d(poseManager.getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
    }
    driver.back().onTrue(runOnce(() -> allowAutoDrive = !allowAutoDrive).ignoringDisable(true));

    // Operator controls

    // State-Based Triggers
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link Autos} class. */
  @Override
  public void autonomousInit() {
    autoCommand = autos.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    driveTest().schedule();
  }

  private Command driveTest() {
    final Timer timer = new Timer();
    final double modifier = 0.2;

    final DoubleSupplier speedSupplier =
        () -> {
          if (timer.hasElapsed(8)) {
            return 0;
          }
          double voltage = (timer.get() + 1) * modifier;
          if (voltage > 10) {
            return -10;
          }
          return voltage;
        };

    return drive
        .moduleTestingCommand(speedSupplier, speedSupplier)
        .beforeStarting(
            () -> {
              drive.allModules = true;
              timer.restart();
            });
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
