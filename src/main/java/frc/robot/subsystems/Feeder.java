package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.RIO_Channels_DIO;

public class Feeder extends SubsystemBase {

    private final CANSparkMax m_Feeder;

    DigitalInput breamBreak = new DigitalInput(RIO_Channels_DIO.FEEDER_BEAM_BREAK);

    public Feeder() {

        m_Feeder = new CANSparkMax(LauncherConstants.kFeeder, MotorType.kBrushless);

        m_Feeder.restoreFactoryDefaults();

        m_Feeder.setInverted(false);

        m_Feeder.setIdleMode(IdleMode.kCoast);

        m_Feeder.setSmartCurrentLimit(60);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Beam Break", isNoteDetected());

    }

    public boolean isNoteDetected() {
        return !breamBreak.get();
    }

    public void stopAll() {
        m_Feeder.set(0);
    }

    public void feed() {
        m_Feeder.set(-.5);
    }

    public void reversefeed() {
        m_Feeder.set(.15);
    }

}
