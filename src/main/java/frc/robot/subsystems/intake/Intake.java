package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInput(inputs);
  }

  public Command command() {
    return run(() -> io.runVolts(0.0));
  }

  public Command intake() {
    return run(() -> io.runVolts(IntakeSpeedVolts));
  }

  public Command eject() {
    return run(() -> io.runVolts(-6.0));
  }
}
