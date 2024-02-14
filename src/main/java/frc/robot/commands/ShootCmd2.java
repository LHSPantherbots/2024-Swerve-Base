package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Feeder;

public class ShootCmd2 extends Command {
    Launcher launcher;
    Feeder feader;
    Leds leds;
    Double curentSpikeLevel = 10.0;
    boolean currentSpikeDetected = false;
    

    public ShootCmd2(Launcher launcher, Leds leds) {
        this.launcher = launcher;
        this.leds = leds;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        this.launcher.lancherMaxSpeed();
        
    }

    @Override
    public void execute() {
        this.launcher.closedLoopLaunch();
        if (this.launcher.isAtVelocity()) {
            this.leds.blue();
        } else {
            // this.feader.stopAll();
        }

        // if (this.launcher.getCurrent() > curentSpikeLevel) {
        //     this.currentSpikeDetected = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.stopLauncher();
        
    }

    @Override
    public boolean isFinished() {
        return this.currentSpikeDetected;
    }

}
