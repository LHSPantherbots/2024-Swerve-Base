package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Feeder extends SubsystemBase {
    
        private final CANSparkMax m_Feeder;


        public Feeder(){
    
        m_Feeder = new CANSparkMax(LauncherConstants.kFeeder, MotorType.kBrushless);


        
        m_Feeder.restoreFactoryDefaults();


      
        m_Feeder.setInverted(false);


       
        m_Feeder.setIdleMode(IdleMode.kCoast);


       
        m_Feeder.setSmartCurrentLimit(60);

   
    }

    @Override
    public void periodic(){

    }

 



    public void stopAll(){
        m_Feeder.set(0);
    }

    public void feed(){
        m_Feeder.set(.5);
    }

    public void reversefeed(){
        m_Feeder.set(-.5);
    }

            

}