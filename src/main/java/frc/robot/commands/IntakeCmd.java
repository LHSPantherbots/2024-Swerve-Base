package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command {
    Intake intake;
    Double curentSpikeLevel = 10.0;
    boolean currentSpikeDetected = false;

    public IntakeCmd(
            Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.intake.intake();
    }

    @Override
    public void execute() {
        if (this.intake.getCurrent() > this.curentSpikeLevel) {
            this.currentSpikeDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
    }

    @Override
    public boolean isFinished() {
        return this.currentSpikeDetected;
    }

}
