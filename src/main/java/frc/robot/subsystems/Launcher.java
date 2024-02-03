package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    
        private final CANSparkFlex m_LauncherTop;
        private final CANSparkFlex m_LauncherBottom;
        private final CANSparkMax m_Feeder;

        private double lastSetpoint = 0;
        private double setPoint = 0;

        public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;
        private SparkPIDController pidController;

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

        pidController = m_LauncherTop.getPIDController();

        // PID coefficients these will need to be tuned
        kP = 0.00015;// 0.00025; //5e-5;
        kI = 0;// 1e-6;
        kD = 0.0008;// 0.0004;
        kIz = 0;
        kFF = 0.00017;// 0.00019;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 100; // 50 //Lets the system known when the velocity is close enough to launch

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void periodic(){

    }

    public void closedLoopLaunch() {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kI);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    
        pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        }

    public void launcherRpmUp(){
        lastSetpoint = setPoint;
        setPoint = setPoint + 250;
        closedLoopLaunch();

    }

    public void launcherRpmDown(){
        lastSetpoint = setPoint;
        setPoint = setPoint -250;
        closedLoopLaunch();
    }

    public void lancherMaxSpeed(){
        lastSetpoint= setPoint;
        setPoint = 6000;
        closedLoopLaunch();
    }

    public void stopLauncher(){
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void stopAll(){
        m_Feeder.set(0);
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void feed(){
        m_Feeder.set(.5);
    }


            

}
