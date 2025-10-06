package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static boolean simHasCoral = false;
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
  private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LinearFilter currentFilter = LinearFilter.movingAverage(4);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    measuredVisualizer.update(Degrees.of(inputs.pivotPositionRads));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpoint", positionSetpoint);
    Util.logSubsystem(this, "Intake");
  }

  public void lowerIntake() {
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  public void raiseIntake() {
    positionSetpoint = raisedAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  public void runRollersIn() {
    io.runRollers(inVoltage.get());
  }

  public void runRollersOut() {
    io.runRollers(-inVoltage.get());
  }

  public void stopRollers() {
    io.runRollers(0);
  }

  public boolean coralHeld() {
    return simHasCoral || inputs.beambreak;
  }

  public Command intake() {
    return run(() -> {
          lowerIntake();
          runRollersIn();
        })
        .until(this::coralHeld)
        .andThen(raiseAndStopCommand())
        .withName("intake");
  }

  public Command raiseAndStopCommand() {
    return run(() -> {
          raiseIntake();
          stopRollers();
        })
        .withName("raise and stop");
  }
}
