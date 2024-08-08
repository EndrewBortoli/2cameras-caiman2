package frc.robot.commands.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.JacLib.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.DriveSubsystem;

public class TurnToSpeaker extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final PIDController controller;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double minVelocity = 0;
  private static double toleranceDegrees = 0;
  private DriverStation.Alliance alliance = null;

  /** Creates a new TurnToSpeaker. Turns to the specified rotation. */
  public TurnToSpeaker(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;

    kP = 0.02;
    kI = 0.0;
    kD = 0.0;
    minVelocity = 0.0;
    toleranceDegrees = 1.0;

    controller = new PIDController(kP, kI, kD, 0.02);
    controller.setTolerance(toleranceDegrees);
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();

    if (DriverStation.getAlliance().isPresent()) {
      this.alliance = DriverStation.getAlliance().get();
    }

    Rotation2d setpoint;
    if (alliance == DriverStation.Alliance.Blue) {
      setpoint = new Rotation2d(
        FieldConstants.FIELD_LENGTH_METERS - driveSubsystem.getPose().getX(),
        FieldConstants.SPEAKER_Y - driveSubsystem.getPose().getY());
    } else {
      setpoint = new Rotation2d(
        driveSubsystem.getPose().getX(),
        FieldConstants.SPEAKER_Y - driveSubsystem.getPose().getY());
    }

    double angleDegrees = setpoint.getDegrees();
    if (alliance == DriverStation.Alliance.Red) {
      angleDegrees = angleDegrees - 180;
    }
    controller.setSetpoint(angleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angularSpeed = controller.calculate(driveSubsystem.getHeading());

    if (Math.abs(angularSpeed) < minVelocity) {
      angularSpeed = Math.copySign(minVelocity, angularSpeed);
    }

    // Use the drive method from the DriveSubsystem to turn the robot
    driveSubsystem.drive(0, 0, angularSpeed, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
