package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteLimeLight;

public class DriveToNote  extends Command {
    Feeder feeder;
    Intake intake;
    DriveSubsystem driveSubsystem;
    NoteLimeLight ll;
    PIDController X_pid;
    PIDController Y_pid;

    public DriveToNote(Feeder feeder, Intake intake, DriveSubsystem driveSubsystem, NoteLimeLight ll) {
        this.feeder = feeder;
        this.intake = intake;
        this.driveSubsystem = driveSubsystem;
        this.ll = ll;
        addRequirements(driveSubsystem, ll);

        X_pid = new PIDController(0, 0, 0);
        X_pid.disableContinuousInput();
        X_pid.setTolerance(10);
        Y_pid = new PIDController(0, 0, 0);
        Y_pid.disableContinuousInput();

        this.raceWith(new IntakeCmd(intake, feeder));
    }

    @Override
    public void initialize() {
        X_pid.reset();
        Y_pid.reset();

    }

    @Override
    public void execute() {
        var y_output = Y_pid.calculate(ll.getHorizontalOffset());
        Double x_output;
        if (Y_pid.atSetpoint()) {
            x_output = X_pid.calculate(ll.getVerticalOffset());
        } else {
            x_output = 0.0;
        }
        driveSubsystem.drive(x_output, y_output, 0, false, false);
        
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return this.ll.isTargetValid();
    }
    
}
