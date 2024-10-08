// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.util.Optional;
import java.util.TimerTask;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.Elevator.Angle.ElevatorAngleChangeSetpointCmd;
import frc.robot.commands.Elevator.Move.ElevatorMoveChangeSetpointCmd;
import frc.robot.commands.launcherjoint.ChangeSetpointLauncherCmd;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.vision.LimelightObject;
import frc.robot.subsystems.vision.PhotonLL;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveSubsystem extends SubsystemBase {
  
  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      true);

   public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      false);

   public final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      true);

   public final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      true);

        //Variaveis criadas para serem usadas no método autoAlign();
      public PhotonLL limelight;


  // The gyro sensor
  final static Pigeon2 m_gyro = new Pigeon2(8);

    private Field2d field = new Field2d();


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private final SwerveDrivePoseEstimator poseEstimator;


    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */


    public Boolean vision = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    this.limelight = PhotonLL.getInstance();
    
                // Define the standard deviations for the pose estimator, which determine how fast the pose
        // estimate converges to the vision measurement. This should depend on the vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose estimator.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        m_gyro.getRotation2d(),
                        getModulePositions(),
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);
                        
            // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));


    SmartDashboard.putData("Field", field);


              //autobuilder needs to be configured last, add anything before this
          AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            //^These commands need to be createds

            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.05, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
                    2, // Max module speed, in m/s
                    0.50, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
            
  );


  }

  @Override
  public void periodic() {

    poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());

            // Correct pose estimate with vision measurements
            var visionEst = limelight.getEstimatedGlobalPose();
            visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = limelight.getEstimationStdDevs(estPose);
    
                        addVisionMeasurement(
                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
    

    // Update the odometry in the periodic block
    // m_odometry.update(
    //     Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()),
    //     new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //     });

    SmartDashboard.putNumber("RotationSpeed", MAXSwerveModule.m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());     

    field.setRobotPose(getPose());

  }

  public void frontLeft() {
   m_frontLeft.getPosition(); 
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

      /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
      public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    /**
     * Reset the estimated pose of the swerve drive on the field.
     *
     * @param pose New robot pose.
     */
    public void resetPose(Pose2d pose) {


        poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }



  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  // public Pose2d getPose() {
  //   return m_odometry.getPoseMeters();
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  //   m_odometry.resetPosition(
  //        Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()),
  //       new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_rearLeft.getPosition(),
  //           m_rearRight.getPosition()
  //       },
  //       pose);
  // }

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


    // System.out.println("Converted Velocities - X: " + xSpeedDelivered + ", Y: " + ySpeedDelivered + ", Rot: " + rotDelivered);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees((m_gyro.getRotation2d().getDegrees())))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
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
    m_gyro.setYaw(0);
  }

  public void rotate(double rotation) {
    // Converta a velocidade de rotação desejada em radianos por segundo
    double rotationalSpeed = rotation * DriveConstants.kMaxAngularSpeed;
  
    // Configura o estado de módulo de cada roda para girar
    SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(rotationalSpeed));

    SwerveModuleState[] states = {desiredState, desiredState, desiredState, desiredState};
    setModuleStates(states);

    // SmartDashboard.putNumber("RotationSpeed", Rotation2d.fromRadians(rotationalSpeed));

    
  }

  public ChassisSpeeds getChassisSpeeds() {
    final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(DriveConstants.globalxSpeed, DriveConstants.globalySpeed, DriveConstants.globalRot);
    return desiredSpeeds;
  }

  //see drive constants for details
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }


  // public void RotationalSpeed(){
  //   double rotationalSpeed = rotation * DriveConstants.kMaxAngularSpeed;

  //   Rotation2d.fromRadians(rotationalSpeed);
  // }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
   public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();  }

    /**
     * Retorna um array contendo as posições dos módulos.
     *
     * @return Um array de objetos SwerveModulePosition.
     */

    public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      };
  }

      /**
     * Retorna um array contendo os estados atuais dos módulos.
     *
     * @return Um array de objetos SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {
      return new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
      };
  }




  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */  
  
  public void autoAlignATest() {
    
    this.limelight = PhotonLL.getInstance();

    
    PIDController rotationPID = null;  
    PIDController carangueijoPID = null;
    PIDController forwardPID = null;
  
    double rotationOffset = 0;
    double carangueijoOffset = -0.25;
    double forwardOffset = 3.2;
  
    final SlewRateLimiter xLimiter, rotLimiter, yLimiter;
  
    rotLimiter = m_rotLimiter;
    xLimiter = m_magLimiter;
    yLimiter = m_magLimiter;

    // Controles PID para se alinhar
    carangueijoPID = new PIDController(0.2, 0, 0);
    rotationPID = new PIDController(0.2, 0, 0);
    forwardPID = new PIDController(0.2, 0, 0);
  
    double velCarangueijo = 0;
    double velRotation = 0;
    double velForward = 0;
    if (limelight.hasValueTargets()) {
        velCarangueijo = carangueijoPID.calculate(limelight.getXDistance(), carangueijoOffset);
        velRotation = rotationPID.calculate(limelight.getYaw(), rotationOffset);
        velForward = forwardPID.calculate(limelight.getArea(), forwardOffset);
            
        if (limelight.getYaw() > 175 && limelight.getYaw() < 5){
          velCarangueijo = 0.0;
          velRotation = 0.0;
          velForward = 0.0;
        }

    } else {
        velCarangueijo = 0.0;
        velRotation = 0.0;
        velForward = 0.0;
    }



    // SmartDashboard.putNumber("DRIVE OUTPUT", velForward);

  
    // 2. Apply deadband
    velCarangueijo = Math.abs(velCarangueijo) > OIConstants.kDriveDeadband ? velCarangueijo : 0.0;
    velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;
    velForward = Math.abs(velForward) > OIConstants.kDriveDeadband ? velForward : 0.0;
  
    // 3. Make the driving smoother
    velForward = yLimiter.calculate(velForward) * 3;
    velCarangueijo = xLimiter.calculate(velCarangueijo) * 3;
    velRotation = rotLimiter.calculate(velRotation) * 5;

    // SmartDashboard.putNumber("giro output", velRotation);
    // SmartDashboard.putNumber("GIRO GOAL", rotationPID.getGoal().position);

  
    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velForward, velCarangueijo, velRotation);
  
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
  }

  public void autoAlignX() {
    
    this.limelight = PhotonLL.getInstance();

    
    PIDController rotationPID = null;  
  
    double rotationOffset = 0;
  
    final SlewRateLimiter xLimiter, rotLimiter, yLimiter;
  
    rotLimiter = m_rotLimiter;
    xLimiter = m_magLimiter;
    yLimiter = m_magLimiter;
  
    // Controles PID para se alinhar
    rotationPID = new PIDController(0.4, 0, 0);
  
    double velCarangueijo = 0;
    double velRotation = 0;
    double velForward = 0;
    if (limelight.hasValueTargets()) {
        velRotation = rotationPID.calculate(limelight.getYaw(), rotationOffset);
            
        if (limelight.getYaw() > 175 && limelight.getYaw() < 5){
          velCarangueijo = 0.0;
          velRotation = 0.0;
          velForward = 0.0;
          System.out.println("Se alinhou");
        }

    } else {
        velCarangueijo = 0.0;
        velRotation = 0.0;
        velForward = 0.0;
    }

    

    // SmartDashboard.putNumber("DRIVE OUTPUT", velForward);

    velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;
  
    // 3. Make the driving smoother
    velRotation = rotLimiter.calculate(velRotation) * 5;

    // SmartDashboard.putNumber("giro output", velRotation);
    // SmartDashboard.putNumber("GIRO GOAL", rotationPID.getGoal().position);

  
    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velForward, velCarangueijo, velRotation);
  
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
  }

  public boolean isAlignedNote() {
    if (m_LimelightObject.getXLimelight() < 5 & m_LimelightObject.getXLimelight() > -5) {

      vision = true;
      return true;

    } else {

      vision = false;
      return false;

    }
  }
  public boolean isAlignedPV() {
    if(limelight.correctID(3)){
      this.limelight = PhotonLL.getInstance();
      if (limelight.getYaw() < 20 & limelight.getYaw() > 10) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
    
  }

 public void autoAlignLeft() {
    
this.limelight = PhotonLL.getInstance();

    
    PIDController rotationPID = null;  
  
    double rotationOffset = 0;

  
    final SlewRateLimiter xLimiter, rotLimiter, yLimiter;
  
    rotLimiter = m_rotLimiter;
    xLimiter = m_magLimiter;
    yLimiter = m_magLimiter;
  
    // Controles PID para se alinhar
  
    double velRotation = 0;


    if (limelight.hasValueTargets()) {
      if(isAlignedPV() == false)
        if(limelight.getYaw() < 20 ) {
        velRotation = 0.1;
        } else if ( limelight.getYaw() > 10) {
          velRotation = -0.1;
        }
    } else {
      velRotation = -0.15;
    }

    // SmartDashboard.putNumber("getXDistance?", limelight.getXDistance());
    // SmartDashboard.putNumber("velRotation?", velRotation);


    velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;
  
    // 3. Make the driving smoother
    velRotation = rotLimiter.calculate(velRotation) * 5;

    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, velRotation);
  
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
  }

  public void autoAlignRight() {
    
    this.limelight = PhotonLL.getInstance();

    
    PIDController rotationPID = null;  
  
    double rotationOffset = 0;

  
    final SlewRateLimiter xLimiter, rotLimiter, yLimiter;
  
    rotLimiter = m_rotLimiter;
    xLimiter = m_magLimiter;
    yLimiter = m_magLimiter;
  
    // Controles PID para se alinhar
  
    double velRotation = 0;


    if (limelight.hasValueTargets()) {
      if(isAlignedPV() == false)
        if(limelight.getYaw() < 20 ) {
        velRotation = 0.1;
        } else if ( limelight.getYaw() > 10) {
          velRotation = -0.1;
        }
    } else {
      velRotation = 0.15;
    }

    // SmartDashboard.putNumber("getXDistance?", limelight.getXDistance());
    // SmartDashboard.putNumber("velRotation?", velRotation);


    velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;
  
    // 3. Make the driving smoother
    velRotation = rotLimiter.calculate(velRotation) * 5;

    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, velRotation);
  
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
  }

  LimelightObject m_LimelightObject = LimelightObject.getInstance();  // Use the existing instance


  
  public void autoAlignNote() {
    

    PIDController rotationPID = null;  
    PIDController forwardPID = null;
  
    double rotationOffset = 0;
    double forwardOffset = 3.2;

    final SlewRateLimiter yLimiter, rotLimiter;
  
    rotLimiter = m_rotLimiter;
    yLimiter = m_magLimiter;

  
    // Controles PID para se alinhar
    rotationPID = new PIDController(1, 0, 0);

  
    double velRotation = 0;
    double velForward = 0;
  
    if (m_LimelightObject.hasObject()) {
        if (!isAlignedNote()) {
            if(m_LimelightObject.getXLimelight() < 0) {
        velRotation = -0.1;
        
      } else {
        velRotation = 0.1;
      }
        } 
        if (isAlignedNote()) {
            velRotation = 0;
          } 
    } else {
      if(m_LimelightObject.getXLimelight() < 0) {
        velRotation = 0.25;
      } else {
        velRotation = -0.25;
      }
    }

    SmartDashboard.putBoolean("objectIsSeen?", m_LimelightObject.objectIsSeen());
    SmartDashboard.putBoolean("hasObject?", m_LimelightObject.hasObject());
    SmartDashboard.putBoolean("isAlignedNote?", isAlignedNote());
    // SmartDashboard.putNumber("velRotation",velRotation);
    // SmartDashboard.putNumber("rotationOffSet",rotationOffset);
    // 2. Apply deadband
    velRotation = Math.abs(velRotation) > OIConstants.kDriveDeadband ? velRotation : 0.0;
    velForward = Math.abs(velForward) > OIConstants.kDriveDeadband ? velForward : 0.0;
  
    // 3. Make the driving smoother
    velRotation = rotLimiter.calculate(velRotation) * 5;
    velForward = yLimiter.calculate(velForward) * 3;

    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velForward, 0, velRotation);
  
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
}

public void stop(){
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    // Set chassis speeds using existing method
    setChassisSpeeds(chassisSpeeds);
}

  public void goLeft() {
    double angle = Math.PI / 2.0; // 90 graus em radianos

    m_frontLeft.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
}

  public void goRight() {
    double angle = Math.PI / -2.0; // 90 graus em radianos

    m_frontLeft.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(1, new Rotation2d(angle)));
}

  public void goForward() {
    double angle = Math.PI / -2.0; // 90 graus em radianos

    m_frontLeft.setDesiredState(new SwerveModuleState(1.5, new Rotation2d(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(1.5, new Rotation2d(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(1.5, new Rotation2d(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(1.5, new Rotation2d(0)));
}

public Rotation2d getGyroscopeRotation() {
  return m_gyro.getRotation2d();

  // We have to invert the angle of the Pigeon so that rotating the robot counter-clockwise makes the angle increase.
  // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
}

private static DriveSubsystem instance;

public static DriveSubsystem getInstance(){
  if(instance == null){
    instance = new DriveSubsystem();
  }
  return instance;
}

  
  
}
