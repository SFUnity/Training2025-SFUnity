package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer("elevator");
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          ElevatorConstants.kP.get(),
          ElevatorConstants.kI.get(),
          ElevatorConstants.kD.get(),
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxElevatorSpeed, ElevatorConstants.maxElevatorAcceleration));
  private final ElevatorFeedforward ffController =
      new ElevatorFeedforward(
          ElevatorConstants.kS.get(), ElevatorConstants.kG.get(), ElevatorConstants.kV.get());

  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
    pid.setTolerance(0.15);

    setDefaultCommand(stow());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    updateTunables();

    io.runVolts(
        pid.calculate(inputs.position.in(Meters), pid.getGoal())
            + ffController.calculate(pid.getSetpoint().velocity));

    elevatorVisualizer.update(inputs.position.in(Meters));

    Logger.recordOutput("Elevator/goal", pid.getGoal().position);
    Util.logSubsystem(this, "Elevator");
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            pid.setPID(
                ElevatorConstants.kP.get(), ElevatorConstants.kI.get(), ElevatorConstants.kD.get()),
        ElevatorConstants.kP,
        ElevatorConstants.kD);
  }

  public boolean atDesiredHeight() {
    return pid.atGoal();
  }

  public Command stow() {
    return run(() -> pid.setGoal(ElevatorConstants.minHeightInches)).withName("readyStow");
  }

  public Command l1() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightL1)).withName("readyL1");
  }

  public Command l2() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightL2)).withName("readyL2");
  }

  public Command l3() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightL3)).withName("readyL3");
  }

  public Command lowAlgae() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightLowAlgae))
        .withName("readyLowAlgae");
  }

  public Command highAlgae() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightHighAlgae))
        .withName("readyHighAlgae");
  }

  public Command processor() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightProcessor))
        .withName("readyProcessor");
  }

  public Command source() {
    return run(() -> pid.setGoal(ElevatorConstants.desiredHeightSource)).withName("readySource");
  }
}
