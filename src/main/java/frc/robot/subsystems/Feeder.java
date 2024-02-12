package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Feeder extends SubsystemBase {

    private final CANSparkMax m_Feeder;
    private final PIDController m_controller;
    
    RelativeEncoder feederEncoder;

    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kIz = 0.0;
    private double allowableError = 0.5;
    private double positionSetpoint = 0.0;

    public Feeder() {

        m_Feeder = new CANSparkMax(LauncherConstants.kFeeder, MotorType.kBrushless);

        m_Feeder.restoreFactoryDefaults();

        m_Feeder.setInverted(false);

        m_Feeder.setIdleMode(IdleMode.kCoast);

        m_Feeder.setSmartCurrentLimit(30);

        this.feederEncoder = m_Feeder.getEncoder();
        m_controller = new PIDController(kP, kI, kD);
        m_Feeder.burnFlash();
        

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder RPM", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Output", m_Feeder.getAppliedOutput());
        SmartDashboard.putNumber("Feeder Current", m_Feeder.getOutputCurrent());

    }

    public void stopAll() {
        m_Feeder.set(0);
    }

    public void feed() {
        m_Feeder.set(.5);
    }

    public void reversefeed() {
        m_Feeder.set(-.5);
    }

    public void closedLoopFeeder() {
        m_Feeder.set(m_controller.calculate(feederEncoder.getPosition(), positionSetpoint)); // is this needed?
      }

    public boolean isNoteDetected() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isNoteDetected'");
    }



}
