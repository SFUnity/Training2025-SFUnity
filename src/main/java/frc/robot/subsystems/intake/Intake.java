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

  public Command intake() {
    return run(
        () -> {
          io.setPivotPosition(loweredPositionDegrees);
          io.runRollers(IntakeSpeedVolts);
        });
  }

  public Command eject() {
    return run(() -> io.runRollers(ejectVolts));
  }

  public Command stow() {
    return run(
        () -> {
          io.setPivotPosition(0);
          io.runRollers(0);
        });
  }
}
