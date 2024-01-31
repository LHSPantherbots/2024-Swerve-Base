package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_IntakeFront;
    private final CANSparkMax m_IntakeBack;

    public Intake(){
        m_IntakeFront = new CANSparkMax(IntakeConstants.kFrontIntake, MotorType.kBrushless);
        m_IntakeBack = new CANSparkMax(IntakeConstants.kBackIntake, MotorType.kBrushless);

        m_IntakeFront.restoreFactoryDefaults();
        m_IntakeBack.restoreFactoryDefaults();

        m_IntakeFront.setInverted(false);
        m_IntakeBack.setInverted(false);

        m_IntakeFront.setIdleMode(IdleMode.kCoast);
        m_IntakeBack.setIdleMode(IdleMode.kCoast);

        m_IntakeFront.setSmartCurrentLimit(60);
        m_IntakeBack.setSmartCurrentLimit(60);

        m_IntakeBack.follow(m_IntakeFront);
    }

    @Override
    public void periodic(){

    }

    public void intake(){
        m_IntakeFront.set(.7);
    }

    public void intakeStop(){
        m_IntakeFront.set(0);
    }

}
