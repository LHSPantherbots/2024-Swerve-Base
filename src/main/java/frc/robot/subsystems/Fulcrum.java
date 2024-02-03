package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FulcrumConstants;
import frc.robot.Constants.LauncherConstants;

public class Fulcrum extends SubsystemBase {
    private final CANSparkMax m_FulcrumRight;
    private final CANSparkMax m_FulcrumLeft;

    private double lastSetpoint = 0;
    private double setPoint = 0;

    public double kP, kI, kD, kIz, kFF, kDt, kMaxOutput, kMinOutput, maxRPM, allowableError;
    private SparkPIDController pidController;

    RelativeEncoder e_FulcrumEncoder;

    private final TrapezoidProfile.Constraints m_Constraints;
    private final ProfiledPIDController m_Controller;


    public Fulcrum(){
        m_FulcrumRight = new CANSparkMax(FulcrumConstants.kFulcrumRight, MotorType.kBrushless);
        m_FulcrumLeft = new CANSparkMax(FulcrumConstants.kFulcrumLeft, MotorType.kBrushless);
         
        m_FulcrumRight.restoreFactoryDefaults();
        m_FulcrumLeft.restoreFactoryDefaults();
         
        m_FulcrumRight.setInverted(false);
        m_FulcrumLeft.setInverted(false);

        m_FulcrumRight.setIdleMode(IdleMode.kBrake);
        m_FulcrumLeft.setIdleMode(IdleMode.kBrake);

        m_FulcrumRight.setSmartCurrentLimit(60);
        m_FulcrumLeft.setSmartCurrentLimit(60);

        m_FulcrumRight.setClosedLoopRampRate(0.25);
        m_FulcrumLeft.setClosedLoopRampRate(0.25);

        m_FulcrumLeft.follow(m_FulcrumRight);

        pidController = m_FulcrumRight.getPIDController();

        e_FulcrumEncoder = m_FulcrumRight.getEncoder();

        // PID coefficients these will need to be tuned
        kP = 0.00015;// 0.00025; //5e-5;
        kI = 0;// 1e-6;
        kD = 0.0008;// 0.0004;
        kIz = 0;
        kFF = 0.00017;// 0.00019;
        kDt = 0.02;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 100; // 50 //Lets the system known when the velocity is close enough to launch

        m_Constraints = new TrapezoidProfile.Constraints(90,90);
        m_Controller = new ProfiledPIDController(kP, kI, kD, m_Constraints, kDt);

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

public double getSetPoint(){
    return setPoint;
}

public double getLastSetPoint(){
    return lastSetpoint;
}

public void setSetPoint(double point){
    lastSetpoint = setPoint;
    setPoint = point;
}

public void setHorizontalHeight(){
    lastSetpoint = setPoint;
    //setPoint = tbd
    closedLoopFulcrum();
}

public void setAmpHeight(){
    lastSetpoint = setPoint;
    //setPoint = tbd
    closedLoopFulcrum();
}

public void setSpeakerHeight(){
    lastSetpoint = setPoint;
    //setPoint = tbd
    closedLoopFulcrum();
}

public void closedLoopFulcrum(){
    m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(), setPoint));
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