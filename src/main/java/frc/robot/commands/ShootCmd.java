package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder;

public class ShootCmd extends Command {
    Launcher launcher;
    Feeder feeder;
    // Double curentSpikeLevel = 10.0;
    // boolean currentSpikeDetected = false;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;


    public ShootCmd(Launcher launcher, Feeder feeder) {
        this.feeder = feeder;
        this.launcher = launcher;
        addRequirements(feeder, launcher);
    }

    @Override
    public void initialize() {
        this.noteWasDetected = false;
        this.shouldEnd = false;
        this.launcher.lancherMaxSpeed();
        this.feeder.stopAll();
    }

    @Override
    public void execute() {
        if (this.feeder.isNoteDetected() && !noteWasDetected) {
            this.noteWasDetected = true;
        } else if (!this.feeder.isNoteDetected() && noteWasDetected) {
            this.shouldEnd = true;
        }
        this.launcher.closedLoopLaunch();
        if (this.launcher.isAtVelocity()) {
            this.feeder.feed();
        }

        // if (this.launcher.getCurrent() > curentSpikeLevel) {
        //     this.currentSpikeDetected = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.stopLauncher();
        this.feeder.stopAll();
        this.noteWasDetected = false;
        this.shouldEnd = false;
    }

    @Override
    public boolean isFinished() {
        // return this.currentSpikeDetected;
        return this.shouldEnd;
    }

}
