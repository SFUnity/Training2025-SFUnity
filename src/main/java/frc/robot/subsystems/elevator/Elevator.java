package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
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

  public boolean setHeight = false;
  public double goalHeightInches = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    pid.setTolerance(elevatorDistanceToleranceInches);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    updateTunables();

    if (setHeight) {
      pid.setGoal(goalHeightInches);

    } else {
      pid.setGoal(0);
    }

    io.runVolts(
        pid.calculate(inputs.position) + ffController.calculate(pid.getSetpoint().velocity));

    meausedVisualizer.update(inputs.position);
    setpointVisualizer.update(Units.inchesToMeters(pid.getGoal().position));

    Logger.recordOutput("Elevator/goal", Units.inchesToMeters(pid.getGoal().position));
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

  public Command enableElevator() {
    return run(() -> setHeight = true).withName("enableElevator");
  }

  public Command disableElevator() {
    return runOnce(() -> setHeight = false).withName("disableElevator");
  }

  public Command request(ElevatorHeight height) {
    return runOnce(() -> goalHeightInches = height.get()).withName("request" + height.toString());
  }
}
