package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
  private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  private double filteredVelocity;
  private double filteredStatorCurrent;
  public static boolean simHasAlgae = false;
  // In rotations
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/loweredAngle", 19);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/raisedAngle", 86);
  // In percent output
  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Intake/rollerSpeedVolts", 10);

  private Angle positionSetpoint = Degrees.zero();

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;

    io.setPID(kP.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get()), kP);

    // Logs
    measuredVisualizer.update(inputs.pivotCurrentPosition);
    setpointVisualizer.update(positionSetpoint);
    Logger.recordOutput("Intake/positionSetpointRadians", positionSetpoint.in(Radians));
    Util.logSubsystem(this, "Intake");
  }

  private void lower() {
    positionSetpoint = Degrees.of(loweredAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = Degrees.of(raisedAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runRollers(rollersSpeed.get());
  }

  private void rollersOut() {
    io.runRollers(-rollersSpeed.get());
  }

  private void rollersStop() {
    io.runRollers(0);
  }

  public Command raiseAndStopCmd() {
    return run(() -> {
          raise();
          rollersStop();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd() {
    return run(() -> {
          lower();
          rollersIn();
        })
        .until(this::algaeHeld)
        .withName("intake");
  }

  public Command poopCmd() {
    return run(() -> {
          raise();
          rollersOut();
        })
        .until(() -> !algaeHeld())
        .withName("poop");
  }

  @AutoLogOutput
  public boolean algaeHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasAlgae;
    }
    return (filteredVelocity <= algaeVelocityThreshold.get()
            && (filteredStatorCurrent >= algaeCurrentThreshold.get())
        || filteredStatorCurrent <= -2);
  }
}
