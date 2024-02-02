package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
//import frc.robot.Constants.ArmPidConstants;


public class Climb extends SubsystemBase  {

    public static boolean climbMode = false;
    
    CANSparkMax m_LeftArm = new CANSparkMax(ClimbConstants.kArmLeft, MotorType.kBrushless);
    CANSparkMax m_RightArm = new CANSparkMax(ClimbConstants.kArmRight, MotorType.kBrushless);

   
    SparkPIDController l_pidController;
    SparkPIDController r_pidController;
    int smartMotionProfile = 1;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxRPM, maxAcc, allowedErr;

    double pid_setPoint = 0;

    public Climb() {
        m_LeftArm.restoreFactoryDefaults();
        m_RightArm.restoreFactoryDefaults();
        m_LeftArm.setInverted(false);
        m_RightArm.setInverted(true);
        m_LeftArm.setSmartCurrentLimit(40);
        m_RightArm.setSmartCurrentLimit(40);
        m_LeftArm.setClosedLoopRampRate(1);
        m_RightArm.setClosedLoopRampRate(1);

        m_LeftArm.setIdleMode(IdleMode.kBrake);
        m_RightArm.setIdleMode(IdleMode.kBrake);

        
    }
    
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbAmp", m_LeftArm.getOutputCurrent());
        SmartDashboard.putNumber("HookAmp", m_RightArm.getOutputCurrent());
       // SmartDashboard.putNumber("Right Arm Pos", r_encoder.getPosition());
       // SmartDashboard.putNumber("Right Arm Vel", r_encoder.getVelocity());
        SmartDashboard.putNumber("Right Arm Current", m_RightArm.getOutputCurrent());
       // SmartDashboard.putNumber("Left Arm Pos", m_LeftArm.getPosition());
       // SmartDashboard.putNumber("Left Arm Vel", m_LeftArm.getVelocity());
        SmartDashboard.putNumber("Left Arm Current", m_LeftArm.getOutputCurrent());
        SmartDashboard.putNumber("Arm Set Point", pid_setPoint);
        SmartDashboard.putBoolean("Climb Mode", climbMode);
    }


    public void manualleftClimb(double move){
        m_LeftArm.set(move);
    }

    public void manualrightClimb(double move){
        m_RightArm.set(move);
    }

    public void manualClimb(double lift, double trim){
        m_RightArm.set(lift - trim);
        m_LeftArm.set(lift + trim);
        // if (!climbMode || (lift > 0.2 || trim > 0.2)){
        //     lift = deadBand(lift);
        //     trim = deadBand(trim);
        //     l_pidController.setReference(lift + trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // l_arm.set(lift + trim);
        //     r_pidController.setReference(lift - trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // r_arm.set(lift - trim);
        // }
        // } if (!climbMode && (lift < 0.2 && trim < 0.2 )) {
        //     lift = deadBand(lift);
        //     trim = deadBand(trim);
        //     l_pidController.setReference(lift + trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // l_arm.set(lift + trim);
        //     r_pidController.setReference(lift - trim, ControlType.kDutyCycle, smartMotionProfile);
        // }
    }


    public void startArmSmartMotion() {
        l_pidController.setReference(pid_setPoint, CANSparkMax.ControlType.kSmartMotion, smartMotionProfile);
        r_pidController.setReference(pid_setPoint, CANSparkMax.ControlType.kSmartMotion, smartMotionProfile);
    }
   

    public void extendArms() {
        
        pid_setPoint+=16;
        
    }

    public void retractArms() {
        
        pid_setPoint-=16;
        
    }

    public void setArmPidSetPoint(double setPoint){
        pid_setPoint=setPoint;
    }

    public void setClimbModeTrue(){
        climbMode = true;
    }

    public void setClimbModeFalse(){
        climbMode = false;

    }
}