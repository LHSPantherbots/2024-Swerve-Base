package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command {
    Intake intake;
    Feeder feeder;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;
    

    public IntakeCmd(
            Intake intake, Feeder feeder) {
        this.intake = intake;
        this.feeder = feeder;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        this.intake.intake();
        this.feeder.feed();
    }

    @Override
    public void execute() {
        if (this.feeder.isNoteDetected() && !noteWasDetected) {
            this.noteWasDetected = true;
        } else if (!this.feeder.isNoteDetected() && noteWasDetected) {
            this.shouldEnd = true;
        }
        if (!noteWasDetected && !shouldEnd) {
            this.intake.intake();
            this.feeder.feed();
        } else if (noteWasDetected && !shouldEnd) {
            this.intake.intakeStop();
            this.feeder.reversefeed();
        } else if (noteWasDetected && shouldEnd) {
            this.intake.intakeStop();
            this.feeder.stopAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
    }

    @Override
    public boolean isFinished() {
        return this.shouldEnd;
    }

}
