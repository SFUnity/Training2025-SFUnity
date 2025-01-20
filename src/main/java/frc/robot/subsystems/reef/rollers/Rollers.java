package frc.robot.subsystems.reef.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private final DigitalInput beamBreak = new DigitalInput(RollersConstants.beamBreakNumber);
  private double filteredVelocity;
  private double filteredStatorCurrent;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);
    Util.logSubsystem(this, "Rollers");
  }

  public boolean coralHeld() {
    return !beamBreak.get();
  }

  public boolean algaeHeld() {

    return (filteredVelocity <= RollersConstants.algaeVelocityThreshold.get()
            && (filteredStatorCurrent >= RollersConstants.algaeCurrentThreshold.get())
        || filteredStatorCurrent <= -2);
  }

  public Command placeCoralAndHighDealgify() {
    return run(() -> {
          io.runVolts(RollersConstants.placeSpeed);
        })
        .withName("placeCoralRollers");
  }

  public Command lowDealgaefy() {
    return run(() -> {
          io.runVolts(RollersConstants.dealgifyingSpeed);
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
          io.runVolts(RollersConstants.intakingSpeed);
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
          io.runVolts(RollersConstants.intakingSpeed);
        })
        .withName("scoreProcessor");
  }
  // TODO: add "until when 2m distance sensor gets set up"
}
