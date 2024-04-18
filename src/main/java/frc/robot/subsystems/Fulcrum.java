package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.FulcrumConstants;

public class Fulcrum extends SubsystemBase {
    private final CANSparkMax m_FulcrumRight;
    private final CANSparkMax m_FulcrumLeft;

    private double lastSetpoint = 0;
    private double trimZero = 3.0;
    private double setPoint = trimZero;
    private double autoAimTrim = 0;

    public double kP, kI, kD, kIz, kFF, kDt, kMaxOutput, kMinOutput, maxRPM, allowableError;
    private SparkPIDController pidController;

    SparkAbsoluteEncoder e_FulcrumEncoder;

    private final TrapezoidProfile.Constraints m_Constraints;
    private final ProfiledPIDController m_Controller;

    private ShuffleboardTab tab = Shuffleboard.getTab("Tuning");
    private GenericEntry sbAngle = tab.add("Fulcrum Angle", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 90))
            .getEntry();

    static InterpolatingDoubleTreeMap kDistanceToFulcrumAngle = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToFulcrumAngle.put(0.0, 9.0);
        kDistanceToFulcrumAngle.put(1.0, 9.0);
        kDistanceToFulcrumAngle.put(2.0, 20.0);
        // kDistanceToFulcrumAngle.put(2.25, 22.0);
        // kDistanceToFulcrumAngle.put(3.0, 28.0);
        // kDistanceToFulcrumAngle.put(3.5, 29.0);
        // kDistanceToFulcrumAngle.put(4.0, 32.5);
        kDistanceToFulcrumAngle.put(2.5, 25.0);
        kDistanceToFulcrumAngle.put(3.0, 27.0);
        kDistanceToFulcrumAngle.put(3.5, 30.0);
        kDistanceToFulcrumAngle.put(4.0, 33.0);
    }

    final DoubleSubscriber distanceSubscriber;

    public Fulcrum() {
        m_FulcrumRight = new CANSparkMax(FulcrumConstants.kFulcrumRight, MotorType.kBrushless);
        m_FulcrumLeft = new CANSparkMax(FulcrumConstants.kFulcrumLeft, MotorType.kBrushless);

        // m_FulcrumRight.restoreFactoryDefaults();
        // m_FulcrumLeft.restoreFactoryDefaults();

        m_FulcrumRight.setInverted(false);
        m_FulcrumLeft.setInverted(false);

        m_FulcrumRight.setIdleMode(IdleMode.kBrake);
        m_FulcrumLeft.setIdleMode(IdleMode.kBrake);

        m_FulcrumRight.setSmartCurrentLimit(40);
        m_FulcrumLeft.setSmartCurrentLimit(40);

        // m_FulcrumRight.setClosedLoopRampRate(0.25);
        // m_FulcrumLeft.setClosedLoopRampRate(0.25);

        m_FulcrumLeft.follow(m_FulcrumRight, true);

        e_FulcrumEncoder = m_FulcrumRight.getAbsoluteEncoder(Type.kDutyCycle);
        e_FulcrumEncoder.setPositionConversionFactor(360.0);

        pidController = m_FulcrumRight.getPIDController();
        pidController.setFeedbackDevice(e_FulcrumEncoder);

        pidController.setP(0.02);
        pidController.setI(0.0);
        pidController.setD(0.0);
        pidController.setFF(0.0);
        pidController.setOutputRange(-1, 1);

        // PID coefficients these will need to be tuned
        kP = 0.02;// 0.00025; //5e-5;
        kI = 0;// 1e-6;
        kD = 0.0000;// 0.0004;
        kIz = 0;
        kFF = 0.0;// 0.00019;
        kDt = 0.02;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 5; // 50 //Lets the system known when the velocity is close enough to launch

        m_Constraints = new TrapezoidProfile.Constraints(180, 180);
        // m_Controller = new ProfiledPIDController(kP, kI, kD, m_Constraints, kDt);
        m_Controller = new ProfiledPIDController(kP, kI, kD, m_Constraints, kDt);

        // set PID coefficients
        // pidController.setP(kP);
        // pidController.setI(kI);
        // pidController.setD(kD);
        // pidController.setIZone(kIz);
        // pidController.setFF(kFF);
        // pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_FulcrumLeft.burnFlash();
        m_FulcrumRight.burnFlash();

        distanceSubscriber = NetworkTableInstance.getDefault().getDoubleTopic("/Distance").subscribe(0.0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Fulcrum is at Set Point", isAtPoint());
        SmartDashboard.putNumber("Fulcrum Velocity", e_FulcrumEncoder.getVelocity());
        SmartDashboard.putNumber("Fulcrum Output", m_FulcrumRight.getAppliedOutput());
        SmartDashboard.putNumber("Fulcrum Setpoint", setPoint);
        SmartDashboard.putNumber("Fulcrum Pos", e_FulcrumEncoder.getPosition());
        SmartDashboard.putBoolean("Fulcrum Down", isFulcurmDown());
        SmartDashboard.putNumber("Auto Angle", getAutoFulcrumAngle());
        SmartDashboard.putNumber("Fulcrum Trim",autoAimTrim);
    }

    public void manualFulcrum(double move) {
        // an if statment may need to be added to keep the fulcrum from going too far in
        // any given direction
        m_FulcrumRight.set(move);
    }

    public void manualFulcrumLeft(double move) {
        m_FulcrumLeft.set(move);
    }

    public void stopFulcrum() {
        m_FulcrumLeft.set(0.0);
        m_FulcrumRight.set(0.0);
    }

    public double getPosition() {
        return e_FulcrumEncoder.getPosition();
    }

    public void resetController() {
        m_Controller.reset(e_FulcrumEncoder.getPosition());
    }

    public boolean isAtPoint() {
        double error = getPosition() - setPoint;
        return (Math.abs(error) < allowableError);
    }

    public void stop() {
        m_FulcrumRight.set(0.0);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getLastSetPoint() {
        return lastSetpoint;
    }

    public void setSetPoint(double point) {
        lastSetpoint = setPoint;
        setPoint = point;
    }

    public void setHorizontalHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void setAmpHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void setSpeakerHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void closedLoopFulcrum() {
        // m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(),
        // setPoint));
        pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    public boolean isFulcurmDown() {
        return (e_FulcrumEncoder.getPosition() < 20.0);
    }

    // calculates estimated fulcrum angle to hit goal with out accounting for the
    // change in height as fulcrum moves
    public void autoAim() {
        // Double distance = SmartDashboard.getNumber("Distance To Target", .5);
        // Double theta = Math.toRadians(55.0); // angle of shooter relative to arm
        // Double Rz = 0.7; // Meters (Shooter Height)
        // Double Tz = 2.05; // Meters (Target Height)
        // Double alpha = Math.atan((Tz-Rz)/(distance)); // Radians (angle to target)
        // Double beta = theta - alpha; // radians (arm angle)
        lastSetpoint = setPoint;
        setPoint = getAutoFulcrumAngle();
        closedLoopFulcrum();
    }

    public double getSbAngle() {
        return sbAngle.getDouble(1.0);
    }

    public double getFulcrumAngleForDistance(double distance) {
        return kDistanceToFulcrumAngle.get(distance);
    }

    public double getAutoFulcrumAngle() {
        return getFulcrumAngleForDistance(distanceSubscriber.get()) + autoAimTrim;
    }

    public void resetTrim() {
        autoAimTrim = trimZero;
    }

    public void trimUp() {
        autoAimTrim += 0.5;
    }

    public void trimDown() {
        autoAimTrim -= 0.5;
    }

    public void setTrim(double trimValue){
        autoAimTrim = trimValue;
    }

}
