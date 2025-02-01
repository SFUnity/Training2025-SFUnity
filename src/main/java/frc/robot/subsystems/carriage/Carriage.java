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

  private static final LoggedTunableNumber highDealgifyTime =
      new LoggedTunableNumber("Carriage/High Dealgaify Time", 1.0);

  public Carriage(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage", inputs);

    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);

    Util.logSubsystem(this, "Carriage");
  }

  @AutoLogOutput
  public boolean coralHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasCoral;
    }
    return inputs.coralHeld;
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

  public Command stop() {
    return run(() -> io.runVolts(0)).withName("stop");
  }

  public Command placeCoral() {
    return run(() -> io.runVolts(placeSpeedVolts.get()))
        .until(() -> !coralHeld())
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
        .until(() -> coralHeld())
        .withName("intakeCoral");
  }

  public Command scoreProcessor() {
    return run(() -> io.runVolts(intakingSpeedVolts.get()))
        .until(() -> !algaeHeld())
        .withName("scoreProcessor");
  }
}
