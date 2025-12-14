package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

}
