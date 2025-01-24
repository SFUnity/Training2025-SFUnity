package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
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

  public double goalHeight = 0;
  public boolean setHeight = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    pid.setTolerance(0.15);

    // setDefaultCommand(goTo(Stow));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    updateTunables();

    io.runVolts(
        pid.calculate(inputs.position.in(Inches))
            + ffController.calculate(pid.getSetpoint().velocity));

    meausedVisualizer.update(inputs.position.in(Meters));
    setpointVisualizer.update(Units.inchesToMeters(pid.getGoal().position));

    Logger.recordOutput("Elevator/goal", Units.inchesToMeters(pid.getGoal().position));
    Util.logSubsystem(this, "Elevator");

    if (setHeight) {
      pid.setGoal(goalHeight);
    } else {
      pid.setGoal(0);
    }
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> pid.setPID(kP.get(), 0, kD.get()), kP, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> ffController = new ElevatorFeedforward(0, kG.get(), kV.get()), kG, kV);
  }

  public boolean atDesiredHeight() {
    return pid.atGoal();
  }

  public Command enableElevator() {
    return run(() -> setHeight = true);
  }

  public Command disableElevator() {
    return run(() -> setHeight = false);
  }

  public Command request(ElevatorHeight height) {
    return run(() -> goalHeight = height.get()).withName("request" + height.toString());
  }
}
