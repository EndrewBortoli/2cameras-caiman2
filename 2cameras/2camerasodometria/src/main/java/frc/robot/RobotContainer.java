
package frc.robot;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
// import frc.robot.auto.sDrive;
import frc.robot.commands.Climber.ClimberLeft.ClimberLeftChangeSetpoint;
import frc.robot.commands.Climber.ClimberLeft.ClimberLeftCmd;
import frc.robot.commands.Climber.ClimberRight.ClimberRightChangeSetpoint;
import frc.robot.commands.Climber.ClimberRight.ClimberRightCmd;
import frc.robot.commands.Elevator.Angle.ElevatorAngleChangeSetpointCmd;
import frc.robot.commands.Elevator.Angle.ElevatorAngleCmd;
import frc.robot.commands.Elevator.Move.ElevatorMoveChangeSetpointCmd;
import frc.robot.commands.Elevator.Move.ElevatorMoveCmd;
import frc.robot.commands.launcher.LaunchAngleCmd;
import frc.robot.commands.launcher.LauncherCmd;
import frc.robot.commands.launcherjoint.ChangeSetpointLauncherCmd;
import frc.robot.commands.launcherjoint.JointLauncherCommand;
import frc.robot.subsystems.Climber.ClimberLeftSubsystem;
import frc.robot.subsystems.Climber.ClimberRightSubsystem;
import frc.robot.subsystems.DriveTrain.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorAngleSubsystem;
import frc.robot.subsystems.Elevator.ElevatorMoveSubsystem;
import frc.robot.subsystems.JointLauncher.JointLauncherSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Led.LedSubsystem;
import frc.robot.subsystems.vision.LaunchAngle;
import frc.robot.subsystems.vision.LimelightObject;


 
public class RobotContainer {

  // Subsistema do robo
  private final static frc.robot.subsystems.DriveTrain.DriveSubsystem m_robotDrive = new frc.robot.subsystems.DriveTrain.DriveSubsystem();
  private final static LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final static JointLauncherSubsystem jointLauncherSubsystem = new JointLauncherSubsystem();
  private final static ElevatorAngleSubsystem elevatorAngleSubsystem = new ElevatorAngleSubsystem();
  private final static ChangeSetpointLauncherCmd ChangeSetpoinLauncherCmd = new ChangeSetpointLauncherCmd(0);
  private final static ElevatorAngleChangeSetpointCmd changeSetpointElevatorAngleCmd = new ElevatorAngleChangeSetpointCmd(0);
  private final static ElevatorMoveSubsystem elevatorMoveSubsystem = new ElevatorMoveSubsystem();                                              
  private final static ElevatorMoveChangeSetpointCmd elevatorMoveChangeSetpointCmd = new ElevatorMoveChangeSetpointCmd(0);   
  private final static LedSubsystem ledSubsystem = new LedSubsystem();
  private final static LaunchAngle launchAngle = new LaunchAngle();
  
  ClimberRightSubsystem climberRightSubsystem = new ClimberRightSubsystem();
  ClimberRightCmd climberRightCmd = new ClimberRightCmd(climberRightSubsystem, ()->ClimberConstants.kRightClimberSetpoint); 

  ClimberLeftSubsystem climberLeftSubsystem = new ClimberLeftSubsystem();
  ClimberLeftCmd climberLeftCmd = new ClimberLeftCmd(climberLeftSubsystem, ()->ClimberConstants.kLeftClimberSetpoint); 

  // Controle do driver
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();


  public RobotContainer() {
  configureButtonBindings();

    // Configure default commands

      SmartDashboard.putData("Autonomous Mode", m_chooser);


    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    /*
     * Outra maneira de definir um autonomous default
     */

    // Register named commands
    NamedCommands.registerCommand("Floor Intake", floorIntake());
    NamedCommands.registerCommand("Subwoofer launch", Subwoofer());
    NamedCommands.registerCommand("Home", Home());
    NamedCommands.registerCommand("Amp Launching", Amp());
    NamedCommands.registerCommand("Podium Launching", podiumLaunching());
    NamedCommands.registerCommand("Launching", launching());
    NamedCommands.registerCommand("FloorIntakeWithInput", floorIntakeWithInput());
    NamedCommands.registerCommand("floorIntakeOnlyIntake", floorIntakeOnlyIntake());
    NamedCommands.registerCommand("launchingReturn", launchingReturn());
    NamedCommands.registerCommand("SubwooferReturn", SubwooferReturn());
    NamedCommands.registerCommand("alignNote", alignNote());


    m_chooser.setDefaultOption("Null", null);

    // m_chooser.addOption("sDrive", new sDrive(m_robotDrive));
    m_chooser.addOption("Reto", new PathPlannerAuto("Reto"));
    m_chooser.addOption("CenterGamePieceAuto", new PathPlannerAuto("CenterGamePieceAuto"));
    m_chooser.addOption("GoOut", new PathPlannerAuto("GoOut"));
    m_chooser.addOption("2 Pieces Center Auto", new PathPlannerAuto("2PiecesCenterAuto"));
    m_chooser.addOption("Amp Side Auto", new PathPlannerAuto("AmpSideAuto"));
    m_chooser.addOption("PodiumSideAuto", new PathPlannerAuto("PodiumSideAutoSecurity"));
    m_chooser.addOption("GoingToCenter", new PathPlannerAuto("GoingToCenter"));
    m_chooser.addOption("3PiecesCenterNoteAndAmpSideNote", new PathPlannerAuto("3PiecesCenterAndAmpSide"));
    m_chooser.addOption("4PiecesPodiumCenterAmpNotes", new PathPlannerAuto("4PiecesPodiumCenterAmpNotes"));
    m_chooser.addOption("2PiecesAmpSideAuto", new PathPlannerAuto("2PiecesAmpSideAuto"));
    m_chooser.addOption("2PiecesPodiumAuto", new PathPlannerAuto("2PiecesPodiumAuto"));
    m_chooser.addOption("WorldChampion", new PathPlannerAuto("WorldChampion"));
  }

  private void configureButtonBindings() {
    jointLauncherSubsystem.setDefaultCommand(new JointLauncherCommand(jointLauncherSubsystem, ()->LauncherConstants.kLauncherJointMotorSetPoint));
    elevatorAngleSubsystem.setDefaultCommand(new ElevatorAngleCmd(elevatorAngleSubsystem, ()->ElevatorConstants.AngulationElevatorSetPoint));
    elevatorMoveSubsystem.setDefaultCommand(new ElevatorMoveCmd(elevatorMoveSubsystem, ()->ElevatorConstants.ElevatorMovementSetPoint));
    climberRightSubsystem.setDefaultCommand(new ClimberRightCmd(climberRightSubsystem, ()->ClimberConstants.kRightClimberSetpoint));
    climberLeftSubsystem.setDefaultCommand(new ClimberLeftCmd(climberLeftSubsystem, ()->ClimberConstants.kLeftClimberSetpoint));

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    new JoystickButton(m_driverController, Joystick.AxisType.kZ.value)
    .whileTrue(new RunCommand(() -> {
      double rotation = m_driverController.getRawAxis(Joystick.AxisType.kZ.value);
      m_robotDrive.rotate(rotation);
    }, m_robotDrive));

  new JoystickButton(m_operatorController, OperatorConstants.kLauncherOutput).whileTrue(new LauncherCmd(launcherSubsystem, "Launch")); // Lança a GamePiece
  new JoystickButton(m_operatorController, OperatorConstants.kLauncherOutput).or(new JoystickButton(m_operatorController, OperatorConstants.kLauncherInput)).or(new JoystickButton(m_operatorController, OperatorConstants.kTriggerActive)).or(new JoystickButton(m_operatorController, OperatorConstants.kTriggerAmp)).onFalse(new LauncherCmd(launcherSubsystem, "Static")); //Se nada pressionado, o Launcher fica parado
  new JoystickButton(m_operatorController, OperatorConstants.kLauncherInput).whileTrue(new LauncherCmd(launcherSubsystem, "Intake"));  
  new JoystickButton(m_operatorController, OperatorConstants.kTriggerActive).whileTrue(new LauncherCmd(launcherSubsystem, "Trigger"));
  new JoystickButton(m_operatorController, OperatorConstants.kTriggerAmp).whileTrue(new LauncherCmd(launcherSubsystem, "Trigger Amp"));
  new JoystickButton(m_operatorController, OperatorConstants.kHome).whileTrue( Home() );
  // new POVButton(m_operatorController, OperatorConstants.kFloorIntake).whileTrue( floorIntake() );
  new POVButton(m_operatorController, OperatorConstants.kSubwoofer).whileTrue( Subwoofer() );
  new POVButton(m_operatorController, OperatorConstants.kAmp).whileTrue( Amp() );
  new POVButton(m_operatorController, OperatorConstants.kFloorIntake).whileTrue( floorIntake() );
  new POVButton(m_operatorController, OperatorConstants.kPodium).whileTrue( podiumLaunching() );
  new POVButton(m_operatorController, OIConstants.A).whileTrue( new RunCommand(() -> jointLauncherSubsystem.definePosition(), jointLauncherSubsystem) );
  new JoystickButton(m_operatorController, OIConstants.LEFT_STICK).whileTrue(new RunCommand(() -> ledSubsystem.setColor("rainbow")));
  new JoystickButton(m_operatorController, OIConstants.RIGHT_STICK).whileTrue(new RunCommand(() -> ledSubsystem.setColor("green")));

  new JoystickButton(m_driverController, OIConstants.LEFT_STICK).whileTrue(new RunCommand(() -> m_robotDrive.goForward(), m_robotDrive)); // GoForward
  new JoystickButton(m_driverController, OIConstants.Left).whileTrue(new RunCommand(() -> m_robotDrive.goLeft(), m_robotDrive)); // Crangueijo direita
  new JoystickButton(m_driverController, OIConstants.Right).whileTrue(new RunCommand(() -> m_robotDrive.goRight(), m_robotDrive));  // Carangueijo esquerda
  new JoystickButton(m_driverController, OIConstants.START).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)); // Zerar o angulo do robô
  new JoystickButton(m_driverController, OIConstants.Y).whileTrue(new RunCommand(() -> m_robotDrive.autoAlignRight(), m_robotDrive)); // Se alinhar pelo Tx
  new JoystickButton(m_driverController, OIConstants.B).whileTrue(new RunCommand(() -> m_robotDrive.autoAlignLeft(), m_robotDrive)); // Se alinhar pelo Tx
  new JoystickButton(m_driverController, OIConstants.X).whileTrue(new RunCommand(() -> m_robotDrive.autoAlignNote(), m_robotDrive)); // Se alinhar por tudo
  
  new JoystickButton(m_driverController, OIConstants.RIGHT_STICK).whileTrue(            
    AutoBuilder.pathfindToPose(
    new Pose2d(14.21, 5.52, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      5.0, 2.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    0, 
    2.0
  ));

}

 private Command Home() {
     return new SequentialCommandGroup(
      new LauncherCmd(launcherSubsystem, "Static"),
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionHome),
      new WaitCommand(0.1),
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionHome),
      new WaitCommand(0.1),
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionHome),
      new WaitCommand(0.15)
    );
 }

  private Command Subwoofer() {
      return new SequentialCommandGroup(
      new ChangeSetpointLauncherCmd(-24.380558013916016),
      new WaitCommand(0.2),
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionSubwooferMore),
      new WaitCommand(0.2),
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionSubWoofer),
      new WaitCommand(0.1),
      
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionSubwooferRight)
      );
 }

  private Command SubwooferReturn() {
      return new SequentialCommandGroup(
      new ChangeSetpointLauncherCmd(-24.380558013916016),
      new WaitCommand(0.2),
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionSubwooferMore),
      new WaitCommand(0.2),
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionSubWooferReturn),
      new WaitCommand(0.1),
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionSubwooferRight)
      );
 }
 
  private Command Amp() {
      return new SequentialCommandGroup(
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionAmp),
      new WaitCommand(0.4),      
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionAmp),
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionAmp)
      );
 }

 public Command floorIntake() {
     return new SequentialCommandGroup(
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionFloorIntake),
      new WaitCommand(0.4),      
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionFloorIntake),
      new WaitCommand(0.25),
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionHome)
    );
 }

 private Command podiumLaunching() {
     return new SequentialCommandGroup(
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionPodium),
      new WaitCommand(0.25),      
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionPodium),
      new WaitCommand(0.25),
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionPodium)
    );
 }

 private Command LimelightLaunching() {
     return new SequentialCommandGroup(
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionLimelight),
      new WaitCommand(0.25),      
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointLimelightPosition),
      new WaitCommand(0.25),
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionLimelight)
    );
 }
 private Command launching() {
     return new SequentialCommandGroup(
      new LauncherCmd(launcherSubsystem, "Intake"),
      new WaitCommand(1),      
      new LauncherCmd(launcherSubsystem, "Trigger"),
      new WaitCommand(1),
      new LauncherCmd(launcherSubsystem, "Static")
      );
 }

 private Command launchingReturn() {
     return new SequentialCommandGroup(
      new LauncherCmd(launcherSubsystem, "Launch"),
      new WaitCommand(0.35),      
      new LauncherCmd(launcherSubsystem, "Intake"),
      new WaitCommand(2),      
      new LauncherCmd(launcherSubsystem, "Trigger"),
      new WaitCommand(1),
      new LauncherCmd(launcherSubsystem, "Static")
      );
 }

  private Command floorIntakeWithInput() {
     return new SequentialCommandGroup(
      new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionFloorIntake),
      new WaitCommand(0.4),      
      new ChangeSetpointLauncherCmd(PositionConstants.launcherJointPositionFloorIntake),
      new WaitCommand(0.25),
      new ElevatorMoveChangeSetpointCmd(PositionConstants.elevatorMovePositionHome)
      // new LauncherCmd(launcherSubsystem, "Intake"),
      // new WaitCommand(2.2),
      // new LauncherCmd(launcherSubsystem, "Static")
    );
 }

  private Command floorIntakeOnlyIntake() {
      return new LauncherCmd(launcherSubsystem, "Launch");
 }

  private Command climberHome() {
     return new ParallelCommandGroup(
      new ClimberLeftChangeSetpoint(0),
      new ClimberRightChangeSetpoint(0)
    );
 }
  private Command climberMaxUp() {
     return new ParallelCommandGroup(
      new ClimberLeftChangeSetpoint(0),
      new ClimberRightChangeSetpoint(0)
    );
 }
 
private Command alignNote() {
  return new RunCommand(() -> m_robotDrive.autoAlignNote(), m_robotDrive);
}

 public Command getAutonomousCommand() {
  return m_chooser.getSelected();
}
  }





