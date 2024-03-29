package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Fulcrum;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.utils.RobotStatus;

public class IntakeCmd2 extends Command {
    Intake intake;
    Feeder feeder;
    Fulcrum fulcrum;
    Leds leds;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;

    public IntakeCmd2(
            Intake intake, Feeder feeder, Fulcrum fulcrum) {
        this.intake = intake;
        this.feeder = feeder;
        this.fulcrum = fulcrum;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
     
    }

    @Override
    public void execute() {
        if (this.fulcrum.isFulcurmDown()){
            this.intake.intake();
            this.feeder.feed();
            // //this.leds.greenPulse();
            // RobotContainer.leds.greenPulse();
            RobotContainer.leds.setRobotStatus(RobotStatus.INTAKE);
        }else{
            this.intake.intakeStop();
            this.feeder.stopAll();
            //this.leds.greenPulse();
            // RobotContainer.leds.greenPulse();
            RobotContainer.leds.setRobotStatus(RobotStatus.INTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
        this.feeder.stopAll();
        //this.leds.orangePulse();
        // RobotContainer.leds.orangePulse();
        if (this.feeder.isNoteDetected()){
            RobotContainer.leds.setRobotStatus(RobotStatus.NOTE_STORED);
        }else{
            RobotContainer.leds.setRobotStatus(RobotStatus.DEFAULT);
        }
        
    }

    @Override
    public boolean isFinished() {
        return this.feeder.isNoteDetected();
    }

}
