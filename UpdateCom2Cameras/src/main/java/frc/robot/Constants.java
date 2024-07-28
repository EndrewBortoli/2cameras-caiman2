// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {



  public static final class OIConstants {

    // =========================== Driver Joystick ===============================
    public static final int kOperatorControllerPort = 1;
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

    // public static final int brake = 3; //B

    public static final int LB = 5; //LB
    public static final int RB = 6; //RB

    public static final int A = 2; //a
    public static final int X = 1; //x
    public static final int B = 3; //b
    public static final int Y = 4; //y
    public static final int RT = 7; //RT
    public static final int LT = 8; //LT
    public static final int START = 10; //y
    public static final int BACK = 9; //y

    public static final int RIGHT_STICK = 12; //Start
    public static final int LEFT_STICK = 11; //Start

    public static final int Up = 0; 
    public static final int Right = 90; 
    public static final int Down = 180; 
    public static final int Left = 270; 

  }

  
  public static final class DriveConstants {

    public static double globalxSpeed;
    public static double globalySpeed;
    public static double globalRot;
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds


    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 6; // radians per second
    public static final double kMagnitudeSlewRate = 4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)
    
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 30;
    public static final int kFrontRightTurningCanId = 20;
    public static final int kRearRightTurningCanId = 40;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.07285;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.1;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }


  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class LauncherConstants {
    // ID's motores da junta do Launcher
    public static final double kLauncherLeftJointMotorId = 21;
    public static final double kLauncherRightJointMotorId = 22;
    public static final double kLauncherUpJointMotorMaxSpeed = 0.3;
    public static final double kLauncherDownJointMotorMaxSpeed = 0.3;
  
    // PID constantes de angulação
    public static final double kPIDAngulationMotorKp = 0.1;
    public static final double kPIDAngulationMotorKi = 0;
    public static final double kPIDAngulationMotorKd = 0;

    public static double kLauncherJointMotorSetPoint = 0;

    public static final double kLauncherMotorId = 24;
    public static final double kTriggerMotorId = 23;

    public static final double kLauncherMotorVelocity = 1;
    public static final double kTriggerMotorVelocity = 1;
    public static final double kIntakeMotorVelocity = 0.8;
    
  }

  public static final class OperatorConstants {
    public static final int kLauncherOutput = OIConstants.RT; // Lança a Game Piece
    public static final int kLauncherInput = OIConstants.LT; // Pega a Game Piece pelo Launcher
    public static final int kTriggerActive = OIConstants.START; // Pega a Game Piece pelo Launcher
    public static final int kTriggerAmp = OIConstants.BACK; // Pega a Game Piece pelo Launcher

    public static final int kElevatorMoveUp = OIConstants.START;
    public static final int kElevatorMoveDown = OIConstants.BACK;

    //     // ===== ELEVATOR ANGLE =====   
    public static final int kElevatorAngleUp = OIConstants.X;
    public static final int kElevatorAngleDown = OIConstants.B;

    public static final int kHome = OIConstants.Y;
    public static final int kFloorIntake = OIConstants.Down;
    public static final int kSubwoofer = OIConstants.Up;
    public static final int kAmp = OIConstants.Left;
    public static final int kPodium = OIConstants.Right;

  }

  // ================== Constantes dos Equipamentos ==================  
  public static class ElevatorConstants {

    // ID motor de Movimento do Elevator
    public static final double kElevatorMovementMotorId = 13;                // Motor de Movimentação
    
    public static double kMoveVelocityMax = 0.1;           // Velocidade máxima do motor de movimentação
  

    // PID de Movimento
    public static final double ElevatorMomeventkP = 0.2;
    public static final double ElevatorMomeventkI = 0;
    public static final double ElevatorMomeventkD = 0;

    
    // ID's motores de Angulação do Elevator
    public static final double kElevatorRightAngulation = 11;     // Motor de Angulação Direito
    public static final double kElevatorLeftAngulation = 12;      // Motor de Angulação Esquerdo

    // Velocidade Máxima dos Motores
    public static double kDownAngulationVelocityMax = 0.3;     // Velocidade máxima dos motores de angulação do Elevador
    public static double kUpAngulationVelocityMax = 0.3;     // Velocidade máxima dos motores de angulação do Elevador


    // PID de angulação
    public static final double AnglekP = 1.85;
    public static final double AnglekI = 0;
    public static final double AnglekD = 0;

    public static double AngulationElevatorSetPoint = 0;


    
    public static double ElevatorMovementSetPoint = 0;
  }

  public static class PositionConstants {

    public static Double elevatorJointPositionUpLimit = -0.05;
    public static Double elevatorJointPositionDownLimit = -1.9;

    public static Double elevatorJointPositionHome = -0.1;
    public static Double elevatorJointPositionFloorIntake = -1.66;
    // public static Double elevatorJointPositionFloorIntake = -1.569350361824036;
    public static Double elevatorJointPositionAmp = -0.4;
    public static Double elevatorJointPositionSelfAngulation = -0.25;
    public static Double elevatorJointPositionSource = 0.0;
    public static Double elevatorJointPositionSubwooferRight = -1.0; // Posição inicial
    public static Double elevatorJointPositionSubwooferMore = -0.80; // Primeira posição
    public static Double elevatorJointPositionPodium = -0.67;
    public static Double elevatorJointPositionMoving = 0.0;
    public static Double elevatorJointLimelightPosition = -0.375876128673553;

    public static Double elevatorMovePositionHome = 0.0;
    public static Double elevatorMovePositionUpLimit = 0.0;
    public static Double elevatorMovePositionDownLimit = 0.0;
    public static Double elevatorMovePositionExtended = 0.0;
    public static Double elevatorMovePositionHalf = 0.0;
    public static Double elevatorMovePositionAmp = 145.0;
    public static Double elevatorMovePositionPodium = 0.0;
    public static Double elevatorMovePositionMoving = 0.0;
    public static Double elevatorMovePositionLimelight = 0.0;
    
    public static Double launcherJointPositionUpLimit = 0.0;
    public static Double launcherJointPositionDownLimit = 0.0;    
    public static Double launcherJointPositionHome = -5.0;
    public static Double launcherJointPositionSubWoofer = -37.66;
    public static Double launcherJointPositionSubWooferReturn = -35.66;
    public static Double launcherJointPositionPodium = -28.0;
    public static Double launcherJointPositionWing = 0.0;
    // public static Double launcherJointPositionSubwoofer = 40.85;
    public static Double launcherJointPositionFloorIntake = -5.3;
    public static Double launcherJointPositionAmp = -1.97;
    public static Double launcherJointPositionMoving = 0.0;
    public static Double launcherJointPositionLimelight = -24.33318519592285;
    // public static Double launcherJointPositionBackLaunching = -30;

  }
  
  public static class ClimberConstants {
    //Id's
    public static final double kRightClimberId = 32;
    public static final double kLeftClimberId = 31;

    //Velocity 
    public static final double kClimberMaxVelocity = 1.8;
    public static final double kClimberMinVelocity = -1.8;

    public static double kRightClimberSetpoint = 0.0;
    public static double kLeftClimberSetpoint = 0.0;

    public static double commonClimberSetPoint = 0.0;

        // PID constantes de angulação
    public static final double kPIDClimberKp = 0.3;
    public static final double kPIDClimberKi = 0;
    public static final double kPIDClimberKd = 0;

    }

        public static class Vision {
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.325, 0.300, 0.16), new Rotation3d(325, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public class VisionConstants{
      public static final Transform3d CAMERA_TO_ROBOT =
    new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
}
  
  }
  





