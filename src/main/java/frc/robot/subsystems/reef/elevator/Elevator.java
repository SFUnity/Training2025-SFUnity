package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

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
          kP.get(),
          kI.get(),
          kD.get(),
          new TrapezoidProfile.Constraints(maxElevatorSpeed, maxElevatorAcceleration));
  private final ElevatorFeedforward ffController =
      new ElevatorFeedforward(kS.get(), kG.get(), kV.get());

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
        hashCode(), () -> pid.setPID(kP.get(), kI.get(), kD.get()), kP, kD);
  }

  public boolean atDesiredHeight() {
    return pid.atGoal();
  }

  public Command stow() {
    return run(() -> pid.setGoal(minHeightInches)).withName("readyStow");
  }

  public Command l1() {
    return run(() -> pid.setGoal(desiredHeightL1)).withName("readyL1");
  }

  public Command l2() {
    return run(() -> pid.setGoal(desiredHeightL2)).withName("readyL2");
  }

  public Command l3() {
    return run(() -> pid.setGoal(desiredHeightL3)).withName("readyL3");
  }

  public Command lowAlgae() {
    return run(() -> pid.setGoal(desiredHeightLowAlgae)).withName("readyLowAlgae");
  }

  public Command highAlgae() {
    return run(() -> pid.setGoal(desiredHeightHighAlgae)).withName("readyHighAlgae");
  }

  public Command processor() {
    return run(() -> pid.setGoal(desiredHeightProcessor)).withName("readyProcessor");
  }

  public Command source() {
    return run(() -> pid.setGoal(desiredHeightSource)).withName("readySource");
  }
}
