package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    
        private final CANSparkFlex m_LauncherTop;
        private final CANSparkFlex m_LauncherBottom;
        private final CANSparkMax m_Feeder;

        private double lastSetpoint = 0;
        private double setPoint = 0;

        public Launcher(){
        m_LauncherTop = new CANSparkFlex(LauncherConstants.kLauncherTop, MotorType.kBrushless);
        m_LauncherBottom = new CANSparkFlex(LauncherConstants.kLauncherBottom, MotorType.kBrushless);
        m_Feeder = new CANSparkMax(LauncherConstants.kFeeder, MotorType.kBrushless);


        m_LauncherTop.restoreFactoryDefaults();
        m_LauncherBottom.restoreFactoryDefaults();
        m_Feeder.restoreFactoryDefaults();


        m_LauncherTop.setInverted(false);
        m_LauncherBottom.setInverted(false);
        m_Feeder.setInverted(false);


        m_LauncherTop.setIdleMode(IdleMode.kBrake);
        m_LauncherBottom.setIdleMode(IdleMode.kBrake);
        m_Feeder.setIdleMode(IdleMode.kCoast);


        m_LauncherTop.setSmartCurrentLimit(60);
        m_LauncherBottom.setSmartCurrentLimit(60);
        m_Feeder.setSmartCurrentLimit(60);


        m_LauncherBottom.follow(m_LauncherTop);
        //m_LauncherTop.setClosedLoopRampRate(0.25);
    }

    @Override
    public void periodic(){

    }

    public void launcherRpmUp(){
        lastSetpoint = setPoint;
        setPoint = setPoint + 250;
        //closedLoopLaunch();

    }

    public void launcherRpmDown(){
        lastSetpoint = setPoint;
        setPoint = setPoint -250;
    }

            

}
