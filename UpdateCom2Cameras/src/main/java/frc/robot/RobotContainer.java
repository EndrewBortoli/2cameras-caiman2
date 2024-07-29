
package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.Climber.ClimberLeft.ClimberLeftCmd;
import frc.robot.commands.Climber.ClimberRight.ClimberRightCmd;
import frc.robot.commands.Elevator.Angle.ElevatorAngleChangeSetpointCmd;
import frc.robot.commands.Elevator.Angle.ElevatorAngleCmd;
import frc.robot.commands.Elevator.Move.ElevatorMoveChangeSetpointCmd;
import frc.robot.commands.Elevator.Move.ElevatorMoveCmd;
import frc.robot.commands.launcher.LauncherCmd;
import frc.robot.commands.launcherjoint.ChangeSetpointLauncherCmd;
import frc.robot.commands.launcherjoint.JointLauncherCommand;
import frc.robot.subsystems.Climber.ClimberLeftSubsystem;
import frc.robot.subsystems.Climber.ClimberRightSubsystem;
import frc.robot.subsystems.Elevator.ElevatorAngleSubsystem;
import frc.robot.subsystems.Elevator.ElevatorMoveSubsystem;
import frc.robot.subsystems.JointLauncher.JointLauncherSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Led.LedSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.JacLib.JoystickOI;


 
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
  
  ClimberRightSubsystem climberRightSubsystem = new ClimberRightSubsystem();
  ClimberRightCmd climberRightCmd = new ClimberRightCmd(climberRightSubsystem, ()->ClimberConstants.kRightClimberSetpoint); 

  ClimberLeftSubsystem climberLeftSubsystem = new ClimberLeftSubsystem();
  ClimberLeftCmd climberLeftCmd = new ClimberLeftCmd(climberLeftSubsystem, ()->ClimberConstants.kLeftClimberSetpoint); 

  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(
    () -> m_robotDrive.m_gyro.getRotation2d(), 
    () -> m_robotDrive.getModulePositions()
);



  // Controle do driver
  Joystick m_operatorController = new Joystick(JoystickOI.kOperatorControllerPort);
  Joystick m_driverController = new Joystick(JoystickOI.kDriverControllerPort);

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


    m_chooser.setDefaultOption("Null", null);

    // m_chooser.addOption("sDrive", new sDrive(m_robotDrive));
    m_chooser.addOption("Reto", new PathPlannerAuto("Reto"));
    m_chooser.addOption("WorldChampion", new PathPlannerAuto("WorldChampion"));

    configureDashboard();

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
                -MathUtil.applyDeadband(m_driverController.getY(), JoystickOI.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), JoystickOI.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(Joystick.AxisType.kZ.value), JoystickOI.kDriveDeadband),
                true, true),
            m_robotDrive));

  new JoystickButton(m_operatorController, OperatorConstants.kLauncherOutput).whileTrue(new LauncherCmd(launcherSubsystem, "Launch")); // Lança a GamePiece
  new JoystickButton(m_operatorController, OperatorConstants.kLauncherOutput).or(new JoystickButton(m_operatorController, OperatorConstants.kLauncherInput)).or(new JoystickButton(m_operatorController, OperatorConstants.kTriggerActive)).or(new JoystickButton(m_operatorController, OperatorConstants.kTriggerAmp)).onFalse(new LauncherCmd(launcherSubsystem, "Static")); //Se nada pressionado, o Launcher fica parado
  new JoystickButton(m_operatorController, OperatorConstants.kLauncherInput).whileTrue(new InstantCommand(()->new LauncherCmd(launcherSubsystem, "Intake")));  
  new JoystickButton(m_operatorController, OperatorConstants.kTriggerActive).whileTrue(new LauncherCmd(launcherSubsystem, "Trigger"));
  new JoystickButton(m_operatorController, OperatorConstants.kTriggerAmp).whileTrue(new LauncherCmd(launcherSubsystem, "Trigger Amp"));
  new JoystickButton(m_operatorController, OperatorConstants.kHome).whileTrue( Home() );
  // new POVButton(m_operatorController, OperatorConstants.kFloorIntake).whileTrue( floorIntake() );
  new POVButton(m_operatorController, OperatorConstants.kSubwoofer).whileTrue( Subwoofer());
  new POVButton(m_operatorController, OperatorConstants.kAmp).whileTrue( Amp() );
  new POVButton(m_operatorController, OperatorConstants.kFloorIntake).whileTrue( floorIntake() );
  new POVButton(m_operatorController, OperatorConstants.kPodium).whileTrue( podiumLaunching() );
  new POVButton(m_operatorController, JoystickOI.A).whileTrue( new RunCommand(() -> jointLauncherSubsystem.definePosition(), jointLauncherSubsystem) );
  new JoystickButton(m_operatorController, JoystickOI.LEFT_STICK).whileTrue(new RunCommand(() -> ledSubsystem.setColor(LedSubsystem.RAINBOW)));
  new JoystickButton(m_operatorController, JoystickOI.RIGHT_STICK).whileTrue(new RunCommand(() -> ledSubsystem.setColor(LedSubsystem.GREEN)));

  new JoystickButton(m_driverController, JoystickOI.LEFT_STICK).whileTrue(new RunCommand(() -> m_robotDrive.goForward(), m_robotDrive)); // GoForward
  new JoystickButton(m_driverController, JoystickOI.START).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)); // Zerar o angulo do robô
  
}

  private void configureDashboard() {
    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");
    poseEstimator.addDashboardWidgets(visionTab);

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

 public Command getAutonomousCommand() {
  return m_chooser.getSelected();
}
  }





