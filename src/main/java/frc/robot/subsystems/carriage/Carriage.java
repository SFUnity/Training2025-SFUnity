package frc.robot.subsystems.carriage;

import static frc.robot.subsystems.carriage.CarriageConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private boolean realCoralHeld = false;

  public boolean realAlgaeHeld = false;

  private static final LoggedTunableNumber highDealgifyTime =
      new LoggedTunableNumber("Carriage/High Dealgaify Time", 1.0);
  private static final LoggedTunableNumber backupForL3Rots =
      new LoggedTunableNumber("Carriage/Backup for L3 Rots", 4);

  public static boolean coralInDanger = false;

  public Carriage(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateCoralStatus();
    Logger.processInputs("Carriage", inputs);

    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);

    if (filteredVelocity <= algaeVelocityThreshold.get()
        && filteredStatorCurrent >= algaeCurrentThreshold.get()) {
      realAlgaeHeld = true;
    }

    Util.logSubsystem(this, "Carriage");

    Logger.recordOutput("Carriage/coralInDanger", coralInDanger);
  }

  public void updateCoralStatus() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      realCoralHeld = simHasCoral;
    } else {
      if (!inputs.beamBreak && !coralPassed) {
        realCoralHeld = false;

      } else if (inputs.beamBreak && !coralPassed && !realCoralHeld) {
        coralPassed = true;
      } else if (!inputs.beamBreak && coralPassed) {
        realCoralHeld = true;
      } else if (realCoralHeld && inputs.beamBreak && coralPassed) {
        coralPassed = false;
      }
    }
  }

  @AutoLogOutput
  public boolean coralHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasCoral;
    }
    return realCoralHeld;
  }

  @AutoLogOutput
  public boolean algaeHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasAlgae;
    }
    return realAlgaeHeld;
  }

  public Command stopOrHold() {
    return run(() -> io.runVolts(algaeHeld() ? holdSpeedVolts.get() : 0)).withName("stop");
  }

  public Command backUpForL3() {
    return run(() -> io.runVolts(backwardsIntakeSpeedVolts.get()))
        .finallyDo(() -> io.runVolts(0))
        .beforeStarting(() -> io.resetEncoder())
        .until(() -> inputs.positionRots >= backupForL3Rots.get())
        .withName("backUpForL3");
  }

  public Command placeCoral() {
    return run(() -> io.runVolts(placeSpeedVolts.get()))
        .until(() -> !realCoralHeld)
        .withName("placeCoral");
  }

  public Command highDealgify() {
    return run(() -> io.runVolts(highDealgifyingSpeedVolts.get()))
        .withTimeout(highDealgifyTime.get())
        .withName("highDealgify");
  }

  public Command lowDealgify() {
    return run(() -> io.runVolts(lowDealgifyingSpeedVolts.get()))
        .until(() -> algaeHeld())
        .withName("lowDealgify");
  }

  public Command intakeCoral() {
    return Commands.either(
            run(() -> io.runVolts(intakingSpeedVolts.get()))
                .until(() -> realCoralHeld)
                .andThen(
                    run(() -> io.runVolts(backwardsIntakeSpeedVolts.get()))
                        .until(() -> inputs.beamBreak)),
            run(() -> io.runVolts(placeSpeedVolts.get())),
            () -> !coralInDanger)
        .withName("intake coral");
  }

  public Command scoreProcessor() {
    return run(() -> io.runVolts(processorSpeedVolts.get()))
        .withTimeout(.25)
        .andThen(() -> realAlgaeHeld = false)
        .withName("scoreProcessor");
  }
}
