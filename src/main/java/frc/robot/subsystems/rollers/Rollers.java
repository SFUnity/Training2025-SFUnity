package frc.robot.subsystems.rollers;

import static frc.robot.subsystems.rollers.RollersConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private final DigitalInput beamBreak = new DigitalInput(beamBreakNumber);
  private double filteredVelocity;
  private double filteredStatorCurrent;

  public Rollers(RollersIO io) {
    this.io = io;

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);
    Util.logSubsystem(this, "Rollers");
  }

  public boolean coralHeld() {
    return !beamBreak.get();
  }

  public boolean algaeHeld() {

    return (filteredVelocity <= algaeVelocityThreshold.get()
            && (filteredStatorCurrent >= algaeCurrentThreshold.get())
        || filteredStatorCurrent <= -2);
  }

  public Command stop() {
    return run(() -> {
          io.runVolts(0);
        })
        .withName("stopRollers");
  }

  public Command placeCoralAndHighDealgify() {
    return run(() -> {
          io.runVolts(placeSpeed);
        })
        .withName("placeCoralRollers");
  }

  public Command lowDealgaefy() {
    return run(() -> {
          io.runVolts(dealgifyingSpeed);
        })
        .until(() -> algaeHeld())
        .andThen(
            run(
                () -> {
                  io.runVolts(0);
                }))
        .withName("dealgaefy");
  }

  public Command intakeCoral() {
    return run(() -> {
          io.runVolts(intakingSpeed);
        })
        .until(() -> coralHeld())
        .andThen(
            run(
                () -> {
                  io.runVolts(0);
                }))
        .withName("intakeCoralRollers");
  }

  public Command scoreProcessor() {
    return run(() -> {
          io.runVolts(intakingSpeed);
        })
        .withName("scoreProcessor");
  }
  // TODO: add "until when 2m distance sensor gets set up"
}
