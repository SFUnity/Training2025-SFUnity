package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
  private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(4);
  private double filteredCurrent;

  private boolean lowered = false;
  private boolean hasAlgae = false;
  public static boolean simHasAlgae = false;

  private final double intakeDelay = .25;
  private final double outtakeDelay = .3;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    filteredCurrent = currentFilter.calculate(inputs.rollersCurrentAmps);
    lowered = inputs.pivotCurrentPositionDeg >= loweredAngle.get() / 2;

    if (inputs.pivotAppliedVolts <= 0.5 && filteredCurrent >= 10) {
      if (lowered) {
        hasAlgae = true;
      } else {
        hasAlgae = false;
      }
    }

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpoint", positionSetpoint);
    Util.logSubsystem(this, "Intake");
  }

  private void lower() {
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = raisedAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runRollers(rollersSpeedIn.get());
  }

  private void rollersOut() {
    io.runRollers(-rollersSpeedOut.get());
  }

  private void rollersStopOrHold() {
    io.runRollers(0);
  }

  public Command raiseAndStopOrHoldCmd() {
    return run(() -> {
          raise();
          rollersStopOrHold();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd() {
    return Commands.waitUntil(this::algaeHeld)
        .andThen(Commands.waitSeconds(intakeDelay))
        .deadlineFor(
            run(
                () -> {
                  lower();
                  rollersIn();
                }))
        .withName("intake");
  }

  public Command poopCmd() {
    return Commands.waitUntil(() -> !algaeHeld())
        .andThen(Commands.waitSeconds(outtakeDelay))
        .deadlineFor(
            run(
                () -> {
                  raise();
                  rollersOut();
                }))
        .withName("poop");
  }

  @AutoLogOutput
  public boolean algaeHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasAlgae;
    }
    return hasAlgae;
  }
}
