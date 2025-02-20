package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L3;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotCommands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorVisualizer meausedVisualizer =
      new ElevatorVisualizer("Meausred", Color.kGreen);
  private final ElevatorVisualizer setpointVisualizer =
      new ElevatorVisualizer("Setpoint", Color.kBlue, 10);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP.get(),
          0,
          kD.get(),
          new TrapezoidProfile.Constraints(maxElevatorSpeed, maxElevatorAcceleration));
  private ElevatorFeedforward ffController = new ElevatorFeedforward(0, kG.get(), kV.get());

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine elevatorRoutine;
  private final PoseManager poseManager;

  private final LoggedTunableNumber safeDropDist = new LoggedTunableNumber("Elevator/SafeDropDist", 0.3);

  public boolean setHeight = false;
  public double goalHeightInches = 0;

  public Elevator(ElevatorIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;

    pid.setTolerance(elevatorDistanceToleranceInches);
    // Create the SysId routine
    elevatorRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runVolts(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    updateTunables();

    if (setHeight
        || poseManager.getDistanceTo(poseManager.closest(RobotCommands.scoreState)) < safeDropDist.get()) {
      if (Carriage.coralInDanger && goalHeightInches < pastL3Height.get()) {
        pid.setGoal(L3.get());
      } else {
        pid.setGoal(goalHeightInches);
      }
    } else {
      if (Carriage.coralInDanger) {
        pid.setGoal(L3.get());
      } else {
        pid.setGoal(0);
      }
    }

    io.runVolts(
        pid.calculate(inputs.position) + ffController.calculate(pid.getSetpoint().velocity));

    meausedVisualizer.update(Units.inchesToMeters(inputs.position));
    setpointVisualizer.update(Units.inchesToMeters(pid.getGoal().position));

    Logger.recordOutput("Elevator/goal", pid.getGoal().position);
    Logger.recordOutput("Elevator/goalHeight", goalHeightInches);
    Logger.recordOutput("Elevator/setHeight", setHeight);
    Util.logSubsystem(this, "Elevator");
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> pid.setPID(kP.get(), 0, kD.get()), kP, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> ffController = new ElevatorFeedforward(0, kG.get(), kV.get()), kG, kV);
  }

  @AutoLogOutput
  public boolean atDesiredHeight() {
    return pid.atSetpoint();
  }

  @AutoLogOutput
  public boolean atGoalHeight() {
    return Util.equalsWithTolerance(
        goalHeightInches, inputs.position, elevatorDistanceToleranceInches);
  }

  @AutoLogOutput
  public boolean pastL3Height() {
    return pastL3Height.get() <= inputs.position;
  }

  public Command enableElevator() {
    return run(() -> setHeight = true).until(this::atGoalHeight).withName("enableElevator");
  }

  public Command disableElevator() {
    return runOnce(() -> setHeight = false).withName("disableElevator");
  }

  public Command request(ElevatorHeight height) {
    return runOnce(
            () -> {
              goalHeightInches = height.get();
              Logger.recordOutput("Elevator/RequestedHeight", height.toString());
            })
        .withName("request" + height.toString());
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.runVolts(-1.0))
        .until(() -> inputs.currentAmps > 40.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

  public Command runSysidCmd() {
    return Commands.sequence(
        runCurrentZeroing(),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .quasistatic(Direction.kForward)
            .until(() -> inputs.position > maxHeightInches - 5),
        this.runOnce(() -> io.runVolts(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine.quasistatic(Direction.kReverse).until(() -> inputs.position < 5),
        this.runOnce(() -> io.runVolts(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine
            .dynamic(Direction.kForward)
            .until(() -> inputs.position > maxHeightInches - 5),
        this.runOnce(() -> io.runVolts(0.0)),
        Commands.waitSeconds(1.0),
        // Stop when we get close to max to avoid hitting hard stop
        elevatorRoutine.dynamic(Direction.kReverse).until(() -> inputs.position < 5));
  }
}
