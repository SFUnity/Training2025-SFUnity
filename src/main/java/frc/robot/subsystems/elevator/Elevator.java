package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
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

    // setDefaultCommand(stow());
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

  

  // public Command stow() {
  //   return run(() -> pid.setGoal(minHeightInches)).withName("readyStow");
  // }

  public Command l1() {
    return run(() -> goalHeight = ElevatorHeights.L1.height).withName("readyL1");
  }

  public Command l2() {
    return run(() -> goalHeight = ElevatorHeights.L2.height).withName("readyL2");
  }

  public Command l3() {
    return run(() -> goalHeight = ElevatorHeights.L3.height).withName("readyL3");
  }

  public Command lowAlgae() {
    return run(() -> goalHeight = ElevatorHeights.AlgaeLow.height).withName("readyLowAlgae");
  }

  public Command highAlgae() {
    return run(() -> goalHeight = ElevatorHeights.AlgaeHigh.height).withName("readyHighAlgae");
  }

  public Command processor() {
    return run(() -> goalHeight = ElevatorHeights.Processor.height).withName("readyProcessor");
  }

  public Command source() {
    return run(() -> goalHeight = ElevatorHeights.Source.height);
  }

  public Command enableElevator(){
    return run(() -> setHeight = true);
  }
  public Command disableElevator(){
    return run(() -> setHeight = false);
  }
}
