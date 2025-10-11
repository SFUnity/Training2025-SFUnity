package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }

  public Command intake() {
    return run(() -> io.runVolts(6.0));
  }

  public Command eject() {
    return run(() -> io.runVolts(-6.0));
  }

  public Command stop() {
    return run(() -> io.runVolts(0.0));
  }
}
