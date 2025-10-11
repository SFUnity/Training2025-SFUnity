package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollersIO.RollersIOInputs;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void updateInputs(RollersIOInputs newInputs) {
    inputs.current = newInputs.current;
    inputs.voltage = newInputs.voltage;
    inputs.positionRad = newInputs.positionRad;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }

  public void runVolts(double volts) {
    inputs.voltage = volts;
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
