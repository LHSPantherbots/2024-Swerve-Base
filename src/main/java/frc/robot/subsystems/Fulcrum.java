package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FulcrumConstants;

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

        m_FulcrumRight.setSmartCurrentLimit(25);
        m_FulcrumLeft.setSmartCurrentLimit(25);

        m_FulcrumRight.setClosedLoopRampRate(0.25);
        m_FulcrumLeft.setClosedLoopRampRate(0.25);

        m_FulcrumLeft.follow(m_FulcrumRight, true);

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
    SmartDashboard.putBoolean("Fulcrum is at Set Point", isAtPoint());
    SmartDashboard.putNumber("Fulcrum Velocity", e_FulcrumEncoder.getVelocity());
}

public void manualFulcrum(double move){
    //an if statment may need to be added to keep the fulcrum from going too far in any given direction
    m_FulcrumRight.set(move);
}

public void manualFulcrumLeft(double move){
    m_FulcrumLeft.set(move);
}

public void stopFulcrum(){
    m_FulcrumLeft.set(0.0);
    m_FulcrumRight.set(0.0);
}

public double getPosition(){
    return e_FulcrumEncoder.getPosition();
}

public void resetController() {
    m_Controller.reset(e_FulcrumEncoder.getPosition());
}

public boolean isAtPoint(){
    double error = getPosition() - setPoint;
    return (Math.abs(error) < allowableError);
}

public void stop(){
    m_FulcrumRight.set(0.0);
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
    //closedLoopFulcrum(); this may or may not be needed
}

public void setAmpHeight(){
    lastSetpoint = setPoint;
    //setPoint = tbd
    //closedLoopFulcrum(); this may or may not be needed
}

public void setSpeakerHeight(){
    lastSetpoint = setPoint;
    //setPoint = tbd
    //closedLoopFulcrum(); this may or may not be needed
}

public void closedLoopFulcrum(){
    m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(), setPoint));
}


}
