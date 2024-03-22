// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.REVPhysicsSim;

public class DriveSubsystem extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2  m_gyro = new Pigeon2(DriveConstants.kPigeonCAN_Id);

 
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //AutoAim PID values
  private double kP = 0.15; //0.05;
  private double kF = 0.05; //0.0125;
  
  private final Field2d m_field = new Field2d();
  private final StructArrayPublisher<SwerveModuleState> publisher;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final LimeLight m_limeLight = new LimeLight();
  private final DoublePublisher distancePublisher;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.reset();

    if (m_limeLight.isTargetValid()) {
      m_poseEstimator = new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          m_limeLight.getBotPose3d().toPose2d());
    } else {
      m_poseEstimator = new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          new Pose2d());
    }

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeed,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(2.0, 0.0, 0.0),
            new PIDConstants(1.75, 0.0, 0.0),
            4.0,
            0.4,
            new ReplanningConfig(true, false)),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    
    SmartDashboard.putData("Field", m_field);
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    distancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/Distance").publish();
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().run();
    }
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
    if (m_limeLight.isTargetValid() && Math.sqrt((getChassisSpeed().vxMetersPerSecond*getChassisSpeed().vxMetersPerSecond)+(getChassisSpeed().vyMetersPerSecond*getChassisSpeed().vyMetersPerSecond)) > 1.0) {
      var tmpPose = m_limeLight.getBotPose3d().toPose2d();
      if (Math
          .abs(m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(tmpPose.getTranslation())) < 0.75) {
        m_poseEstimator.addVisionMeasurement(m_limeLight.getBotPose3d().toPose2d(), Timer.getFPGATimestamp()
            - (m_limeLight.getTargetLatency() / 1000.0) - (m_limeLight.getCaptureLatency() / 1000.0));
      }

    }
    
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Front Left Angle", m_frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle", m_frontRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Rear Left Angle", m_rearLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Rear Right Angle", m_rearRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Gyro Angle", getAngle());
    SmartDashboard.putString("Chassis Speed", getChassisSpeed().toString());
    publisher.set(new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    });
    distancePublisher.set(getDistanceToTarget());
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    SmartDashboard.putNumber("desired Angle", angleToTarget());
    SmartDashboard.putNumber("auto aim error", errorToTarget());
    SmartDashboard.putNumber("currAngle", currAngle());
    SmartDashboard.putNumber("desired Angle (deg)", Math.toDegrees(angleToTarget()));
    SmartDashboard.putNumber("auto aim error (deg)", Math.toDegrees(errorToTarget()));
    SmartDashboard.putNumber("currAngle (deg)", Math.toDegrees(currAngle()));
    SmartDashboard.putNumber("FR Drive Current", m_frontRight.getDriveCurrent());
    SmartDashboard.putNumber("FL Drive Current", m_frontLeft.getDriveCurrent());
    SmartDashboard.putNumber("RR Drive Current", m_rearRight.getDriveCurrent());
    SmartDashboard.putNumber("RL Drive Current", m_rearLeft.getDriveCurrent());
    SmartDashboard.putNumber("FR Turn Current", m_frontRight.getTurnCurrent());
    SmartDashboard.putNumber("FL Turn Current", m_frontLeft.getTurnCurrent());
    SmartDashboard.putNumber("RR Turn Current", m_rearRight.getTurnCurrent());
    SmartDashboard.putNumber("RL Turn Current", m_rearLeft.getTurnCurrent());
  }


  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
    return states;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (isRed()) {
      Rotation2d rot = pose.getRotation();
      rot.rotateBy(new Rotation2d(Math.toRadians(180.0)));
      pose = new Pose2d(pose.getTranslation(), rot);
    }
    m_poseEstimator.resetPosition(
        // getYaw(),
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
          pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds cs) {
    drive(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond, false, false);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  
  
  public double getAngle(){
    double ang = Math.IEEEremainder(m_gyro.getAngle(), 360);//Returns heading 180 to -180.  Right turn is negative and Left turn is positive
    return ang * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      return false;
    }
  }

  public double angleToTarget() {
    double Ry = m_poseEstimator.getEstimatedPosition().getY();
    double Rx = m_poseEstimator.getEstimatedPosition().getX();
    double desiredAngle = 0;
    double Ty = 5.55;
    double Tx;
    if (isRed()) {
      // Ty = 2.65;
      Tx = 16.579342;
    } else {
      // Ty = 5.55;
      Tx = 0.0;
    }
    if (Ry > Ty) {
      if (isRed()) {
        // desiredAngle = (Math.atan((Ry - Ty)/(Tx - Rx)) * (isRed() ? -1.0 : 1.0)) + Math.PI;
        desiredAngle = Math.atan((Ry - Ty)/(Tx - Rx)) + Math.PI;
        // desiredAngle = Math.IEEEremainder(desiredAngle, 2*Math.PI);
      } else {
        desiredAngle = Math.atan((Ry - Ty)/Rx);
      }
      
    } else {
      if (isRed()) {
        // desiredAngle = (-Math.atan((Ty-Ry)/(Tx - Rx)) * (isRed() ? -1.0 : 1.0)) + Math.PI;
        desiredAngle = Math.atan((Ty-Ry)/(Tx - Rx)) + Math.PI;

        // desiredAngle = Math.IEEEremainder(desiredAngle, 2*Math.PI);
      } else {
        desiredAngle = -Math.atan((Ty-Ry)/Rx);
      }
      
    }
    return Math.IEEEremainder(desiredAngle, 2*Math.PI);
  }

  // public double autoAngleToTarget() {
  //   double Ry = m_poseEstimator.getEstimatedPosition().getY();
  //   double Rx = m_poseEstimator.getEstimatedPosition().getX();
  //   double desiredAngle = 0;
  //   double Ty = 5.55;
  //   double Tx;
  //   if (isRed()) {
  //     // Ty = 2.65;
  //     Tx = 10; // not correct
  //   } else {
  //     // Ty = 5.55;
  //     Tx = 0.0;
  //   }
  //   if (Ry > Ty) {
  //     if (isRed()) {
  //       desiredAngle = (Math.atan((Ry - Ty)/(Tx - Rx)) * (isRed() ? -1.0 : 1.0)) + Math.PI;
  //       Math.IEEEremainder(desiredAngle, 2*Math.PI);
  //     } else {
  //       desiredAngle = Math.atan((Ry - Ty)/Rx) * (isRed() ? -1.0 : 1.0);
  //     }
      
  //   } else {
  //     if (isRed()) {
  //       desiredAngle = (-Math.atan((Ty-Ry)/(Tx - Rx)) * (isRed() ? -1.0 : 1.0)) + Math.PI;

  //       Math.IEEEremainder(desiredAngle, 2*Math.PI);
  //     } else {
  //       desiredAngle = -Math.atan((Ty-Ry)/Rx) * (isRed() ? -1.0 : 1.0);
  //     }
      
  //   }
  //   return desiredAngle;
  // }

  public double errorToTarget() {
    double error = angleToTarget() - currAngle();
    return -error;
  }

  public double currAngle() {
    return m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
  }

  public void autoAim() { 
    double error = angleToTarget() - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    kF = Math.copySign(kF, error);
    double outF = kF;
    double outP = kP * error;
    double outputTurn = outF + outP;
    if (Math.abs(error) > 0.1 ) { // if error is greater than ~5.7 deg (0.1 rad)
      drive(0, 0, outputTurn, false, false);
    } else {
      drive(0, 0, 0, false, false);
    }
  }
  
  public void autoAimAndDrive(Double x, Double y) { 
    double error = angleToTarget() - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    kF = Math.copySign(kF, error);
    double outF = kF;
    double outP = kP * error;
    double outputTurn = outF + outP;
    if (Math.abs(error) > 0.1 ) { // if error is greater than ~5.7 deg (0.1 rad)
      drive(x, y, outputTurn, false, false);
    } else {
      drive(x, y, 0, false, false);
    }
  }

  public void turnToAmpAndDrive(Double x, Double y) {
    double error;
    if (isRed()) {
      error = Math.toRadians(90) - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    } else {
      error = Math.toRadians(-90) - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    }
    kF = Math.copySign(kF, error);
    double outF = kF;
    double outP = kP * error;
    double outputTurn = outF + outP;
    if (Math.abs(error) > 0.1 ) { // if error is greater than ~5.7 deg (0.1 rad)
      drive(x, y, outputTurn, false, false);
    } else {
      drive(x, y, 0, false, false);
    }

  }

  public boolean isAimedAtGoal() {
    double error = angleToTarget() - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    if (Math.abs(error) > 0.1 ) {
      return false;
    } else {
      return true;
    }
  }

  public double getDistanceToTarget() {
    double Ty = 5.55;
    double Tx;
    if (isRed()) {
      Tx = 16.579342;
    } else {
      Tx = 0.0;
    }
    return m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(new Translation2d(Tx, Ty));

  }

  public Rotation2d getYaw() {
    // double yawRadians = Math.toRadians(m_gyro.getYaw().getValueAsDouble());
    // return new Rotation2d(yawRadians);
    return m_gyro.getRotation2d();
  }

  public double getHeadingRadians() {
    return getYaw().getRadians();
  }

  public void restOdomWithCamData() {
    m_poseEstimator.resetPosition(
        new Rotation2d(getHeadingRadians()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        m_limeLight.getLastPose3d().toPose2d());
  }

}
