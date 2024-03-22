package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTargetCmd extends Command {
    DriveSubsystem drive;

    public TurnToTargetCmd(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.drive.drive(0, 0, 0, false, false);
    }

    @Override
    public void execute() {
        //this.drive.restOdomWithCamData();
        this.drive.autoAim();
    }

    @Override
    public void end(boolean interrupted) {
        this.drive.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return this.drive.isAimedAtGoal();
    }
    
}
