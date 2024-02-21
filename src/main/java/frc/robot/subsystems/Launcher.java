package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

    private final CANSparkFlex m_LauncherTop;
    private final CANSparkFlex m_LauncherBottom;
    private final RelativeEncoder m_LauncherEncoder;

    private double lastSetpoint = 0;
    private double setPoint = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;
    private SparkPIDController pidController;

    static InterpolatingDoubleTreeMap kDistanceToShooterSpeed = new InterpolatingDoubleTreeMap();

    // These values are made up, must get real values from testing
    static {
        kDistanceToShooterSpeed.put(0.0, 5000.0);
        kDistanceToShooterSpeed.put(1.0, 5200.0);
        kDistanceToShooterSpeed.put(3.0, 5400.0);
        kDistanceToShooterSpeed.put(5.0, 6000.0);
    }

    final DoubleSubscriber distanceSubscriber;

    public Launcher() {
        m_LauncherTop = new CANSparkFlex(LauncherConstants.kLauncherTop, MotorType.kBrushless);
        m_LauncherBottom = new CANSparkFlex(LauncherConstants.kLauncherBottom, MotorType.kBrushless);

        m_LauncherTop.restoreFactoryDefaults();
        m_LauncherBottom.restoreFactoryDefaults();

        m_LauncherTop.setInverted(false);
        m_LauncherBottom.setInverted(false);

        m_LauncherTop.setIdleMode(IdleMode.kBrake);
        m_LauncherBottom.setIdleMode(IdleMode.kBrake);

        m_LauncherTop.setSmartCurrentLimit(60);
        m_LauncherBottom.setSmartCurrentLimit(60);

        m_LauncherBottom.follow(m_LauncherTop);
        // m_LauncherTop.setClosedLoopRampRate(0.25);

        m_LauncherEncoder = m_LauncherTop.getEncoder();

        pidController = m_LauncherTop.getPIDController();

        // PID coefficients these will need to be tuned
        kP = 0.00015;// 0.00025; //5e-5;
        kI = 0;// 1e-6;
        kD = 0.0008;// 0.0004;
        kIz = 0;
        kFF = 0.00017;// 0.00019; TODO need to tune this
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 250; // 50 //Lets the system known when the velocity is close enough to launch

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        distanceSubscriber = NetworkTableInstance.getDefault().getDoubleTopic("/Distance").subscribe(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current", m_LauncherTop.getOutputCurrent());
        SmartDashboard.putNumber("Launcer RPM", m_LauncherEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher SetPoint", setPoint);
        SmartDashboard.putBoolean("Launcher Is At Vel", isAtVelocity());
        SmartDashboard.putNumber("Auto RPM", getAutoShooterSpeed());
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

    public void launcherRpmUp() {
        lastSetpoint = setPoint;
        setPoint = setPoint + 250;
        closedLoopLaunch();

    }

    public void launcherRpmDown() {
        lastSetpoint = setPoint;
        setPoint = setPoint - 250;
        closedLoopLaunch();
    }

    public void lancherMaxSpeed() {
        lastSetpoint = setPoint;
        setPoint = 5000;
        closedLoopLaunch();
    }

    public void stopLauncher() {
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void stopAll() {
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void resumeLauncher() {
        var tmp = setPoint;
        setPoint = lastSetpoint;
        lastSetpoint = tmp;
        closedLoopLaunch();
    }

    public boolean isAtVelocity() {
        double error = m_LauncherEncoder.getVelocity() - setPoint;
        return (Math.abs(error) < allowableError);
    }

    public double getCurrent() {
        return m_LauncherTop.getOutputCurrent();
    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed.get(distance);
    }

    public double getAutoShooterSpeed() {
        return getShooterSpeedForDistance(distanceSubscriber.get());
    }

}
