package frc.robot.commands.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain.DriveSubsystem;
import frc.JacLib.utils.PhotonLL;
import frc.JacLib.JoystickOI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignToAprilTag extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final PhotonLL limelight;
  private final SlewRateLimiter rotLimiter;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private double velRotation;

  public AlignToAprilTag(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limelight = PhotonLL.getInstance();
    this.rotLimiter = driveSubsystem.m_rotLimiter;
    this.xLimiter = driveSubsystem.m_magLimiter;
    this.yLimiter = driveSubsystem.m_magLimiter;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    velRotation = 0;
  }

  @Override
  public void execute() {
    if (limelight.hasValueTargets()) {
      if (!isAlignedPV()) {
        if (limelight.getYaw() < 20) {
          velRotation = 0.1;
        } else if (limelight.getYaw() > 10) {
          velRotation = -0.1;
        }
      } else {
        velRotation = 0;
      }
    } else {
      velRotation = -0.15;
    }

    SmartDashboard.putNumber("getXDistance?", limelight.getXDistance());
    SmartDashboard.putNumber("velRotation?", velRotation);

    velRotation = Math.abs(velRotation) > JoystickOI.kDriveDeadband ? velRotation : 0.0;
    velRotation = rotLimiter.calculate(velRotation) * 5;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, velRotation);
    driveSubsystem.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return isAlignedPV();
  }

  private boolean isAlignedPV() {
    if (limelight.correctID(3)) {
      if (limelight.getYaw() < 20 && limelight.getYaw() > 10) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
