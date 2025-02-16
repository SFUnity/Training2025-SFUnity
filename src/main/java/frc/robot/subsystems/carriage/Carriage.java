package frc.robot.subsystems.carriage;

import static frc.robot.subsystems.carriage.CarriageConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Carriage extends SubsystemBase {
  private final CarriageIO io;
  private final CarrageIOInputsAutoLogged inputs = new CarrageIOInputsAutoLogged();

  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double filteredVelocity;
  private double filteredStatorCurrent;

  public static boolean simHasCoral = false;
  public static boolean simHasAlgae = false;

  private boolean coralPassed = false;

  public boolean realCoralHeld = false;

  private static final LoggedTunableNumber highDealgifyTime =
      new LoggedTunableNumber("Carriage/High Dealgaify Time", 1.0);

  public Carriage(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    coralHeld();
    Logger.processInputs("Carriage", inputs);

    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);

    Util.logSubsystem(this, "Carriage");

    Logger.recordOutput("Carriage/coralPassed", coralPassed);
    Logger.recordOutput("Carriage/coralHeld", realCoralHeld);
  }

  public void coralHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      realCoralHeld = simHasCoral;
    } else {
      if (!inputs.beamBreak && !coralPassed) {
        realCoralHeld = false;

      } else if (inputs.beamBreak && !coralPassed && !realCoralHeld) {
        coralPassed = true;
      } else if (!inputs.beamBreak && coralPassed) {
        realCoralHeld = true;
      } else if (realCoralHeld && !inputs.beamBreak && coralPassed) {
        realCoralHeld = false;
        coralPassed = false;
      }
    }
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

  public Command stopOrHold() {
    return run(() -> io.runVolts(algaeHeld() ? holdSpeedVolts.get() : 0)).withName("stop");
  }

  public Command placeCoral() {
    return run(() -> io.runVolts(placeSpeedVolts.get()))
        .until(() -> !realCoralHeld)
        .withName("placeCoral");
  }

  public Command highDealgify() {
    return run(() -> io.runVolts(dealgifyingSpeedVolts.get()))
        .withTimeout(highDealgifyTime.get())
        .withName("highDealgify");
  }

  public Command lowDealgify() {
    return run(() -> io.runVolts(dealgifyingSpeedVolts.get()))
        .until(() -> algaeHeld())
        .withName("lowDealgify");
  }

  public Command intakeCoral() {
    return run(() -> io.runVolts(intakingSpeedVolts.get()))
        .until(() -> realCoralHeld)
        .andThen(() -> io.runVolts(intakingSpeedVolts.get() * -1))
        .until(() -> inputs.beamBreak)
        .withName("intake coral");
  }

  public Command scoreProcessor() {
    return run(() -> io.runVolts(intakingSpeedVolts.get()))
        .until(() -> !algaeHeld())
        .withName("scoreProcessor");
  }
}
