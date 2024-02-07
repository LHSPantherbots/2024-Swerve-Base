package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder;

public class ShootCmd extends Command {
    Launcher launcher;
    Feeder feader;
    Double curentSpikeLevel = 10.0;
    boolean currentSpikeDetected = false;

    public ShootCmd(Launcher launcher, Feeder feeder) {
        this.feader = feeder;
        this.launcher = launcher;
        addRequirements(feeder, launcher);
    }

    @Override
    public void initialize() {
        this.launcher.lancherMaxSpeed();
        this.feader.stopAll();
    }

    @Override
    public void execute() {
        this.launcher.closedLoopLaunch();
        if (this.launcher.isAtVelocity()) {
            this.feader.feed();
        } else {
            this.feader.stopAll();
        }

        if (this.launcher.getCurrent()>curentSpikeLevel) {
            this.currentSpikeDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.stopLauncher();
        this.feader.stopAll();
    }

    @Override
    public boolean isFinished() {
        return this.currentSpikeDetected;
    }
    
}
