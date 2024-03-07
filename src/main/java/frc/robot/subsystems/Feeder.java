package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RIO_Channels_DIO;

public class Feeder extends SubsystemBase {

    private final CANSparkMax m_Feeder;
    private final PIDController m_controller;

    private XboxController m_driverFeedBack = new XboxController(OIConstants.kDriverControllerPort);
    private XboxController m_operatorFeedBack = new XboxController(OIConstants.kOperatorControllerPort);

    private boolean lastBeamBreakState;
    private int feedbackCountDown = 0;
    RelativeEncoder feederEncoder;

    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kIz = 0.0;
    private double allowableError = 0.5;
    private double positionSetpoint = 0.0;
  
    DigitalInput breamBreak_raw = new DigitalInput(RIO_Channels_DIO.FEEDER_BEAM_BREAK);
    Debouncer beamBreak = new Debouncer(0.1, DebounceType.kBoth);

    public Feeder() {

        
        m_Feeder = new CANSparkMax(LauncherConstants.kFeeder, MotorType.kBrushless);

        m_Feeder.restoreFactoryDefaults();

        m_Feeder.setInverted(false);

        m_Feeder.setIdleMode(IdleMode.kBrake);

        m_Feeder.setSmartCurrentLimit(30);

        this.feederEncoder = m_Feeder.getEncoder();
        m_controller = new PIDController(kP, kI, kD);
        m_Feeder.burnFlash();

        lastBeamBreakState = !breamBreak_raw.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Beam Break", isNoteDetected());
        SmartDashboard.putNumber("Feeder RPM", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Output", m_Feeder.getAppliedOutput());
        SmartDashboard.putNumber("Feeder Current", m_Feeder.getOutputCurrent());
        if (isNoteDetected() && lastBeamBreakState==false) {
            m_driverFeedBack.setRumble(RumbleType.kBothRumble, 1.0);
            m_operatorFeedBack.setRumble(RumbleType.kBothRumble, 1.0);
            feedbackCountDown = 10;
        }
        if (feedbackCountDown >= 0) {
            feedbackCountDown-=1;
        }
        if (feedbackCountDown == 0) {
            m_driverFeedBack.setRumble(RumbleType.kBothRumble, 0.0);
            m_operatorFeedBack.setRumble(RumbleType.kBothRumble, 0.0);
        }
        lastBeamBreakState = isNoteDetected();

    }

    public boolean isNoteDetected() {
        return !beamBreak.calculate(breamBreak_raw.get());
    }

    public void stopAll() {
        m_Feeder.set(0);
    }

    public void feed() {
        m_Feeder.set(-.5);
    }

    public void setFeed(double feedVal){
        m_Feeder.set(feedVal);
    }

    public void reversefeed() {
        m_Feeder.set(.15);
    }

    public void closedLoopFeeder() {
        m_Feeder.set(m_controller.calculate(feederEncoder.getPosition(), positionSetpoint)); // is this needed?
      }

    public void resetEncoder() {
        feederEncoder.setPosition(0.0);
      }

}
