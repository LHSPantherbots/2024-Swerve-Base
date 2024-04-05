package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Fulcrum;
import frc.robot.subsystems.Launcher;
import frc.utils.Position;
import frc.robot.subsystems.Feeder;

public class AutoShootAndFulcrum extends Command {
    Fulcrum fulcrum;
    Launcher launcher;
    Feeder feeder;
    // boolean noteWasDetected = false;
    boolean shouldEnd = false;
    private boolean startCountDown = false;
    private int countDown = 50;

    public AutoShootAndFulcrum(Fulcrum fulcrum, Launcher launcher, Feeder feeder) {
        this.fulcrum = fulcrum;
        this.feeder = feeder;
        this.launcher = launcher;
        addRequirements(fulcrum, feeder, launcher);
    }

    @Override
    public void initialize() {
        this.fulcrum.resetController();
        
        this.fulcrum.setSetPoint(this.fulcrum.getAutoFulcrumAngle());
        
        // this.noteWasDetected = false;
        this.shouldEnd = false;
        this.startCountDown = false;
        this.countDown = 75;
        this.launcher.setSetPoint(this.launcher.getAutoShooterSpeed());
        this.feeder.stopAll();
    }

    @Override
    public void execute() {
        this.fulcrum.closedLoopFulcrum();
        this.launcher.closedLoopLaunch();

        if (this.launcher.isAtVelocity() && this.fulcrum.isAtPoint()) {
            this.feeder.feed();
            this.startCountDown = true;
        } else {
            this.feeder.stopAll();
        }

        if (this.startCountDown) {
            this.countDown -= 1;
        }

        if (this.countDown <= 0) {
            this.fulcrum.setSetPoint(12);
            this.fulcrum.closedLoopFulcrum();
            this.shouldEnd = true;
        }

        // TODO add count down before allowing this to end
        // if (this.feeder.isNoteDetected() && !noteWasDetected) {
        //     this.noteWasDetected = true;
        // } else if (!this.feeder.isNoteDetected() && noteWasDetected) {
        //     this.shouldEnd = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        this.fulcrum.closedLoopFulcrum();
        this.launcher.stopLauncher();
        this.feeder.stopAll();
        // this.noteWasDetected = false;
        this.shouldEnd = false;
        this.startCountDown = false;
        this.countDown = 50;
    }

    @Override
    public boolean isFinished() {
        // return this.fulcrum.isAtPoint();
        return this.shouldEnd;
    }
}
