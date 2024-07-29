// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.JointLauncher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

public class JointLauncherSubsystem extends SubsystemBase {

  public final static CANSparkFlex jointLauncherLeftMotor = new CANSparkFlex((int) LauncherConstants.kLauncherLeftJointMotorId, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkFlex jointLauncherRightMotor = new CANSparkFlex((int) LauncherConstants.kLauncherRightJointMotorId, CANSparkLowLevel.MotorType.kBrushless);
 
  private RelativeEncoder jointLauncherRightEncoder = jointLauncherRightMotor.getEncoder();
  private RelativeEncoder jointLauncherLeftEncoder = jointLauncherLeftMotor.getEncoder();

  private SparkPIDController leftSparkPIDController = jointLauncherLeftMotor.getPIDController();
  private SparkPIDController rightSparkPIDController = jointLauncherRightMotor.getPIDController();
  PhotonPipelineResult result = new PhotonPipelineResult();

  public double getJointLauncherRightMotorPosition(){
    return jointLauncherRightEncoder.getPosition();
  }
  public double getJointLauncherLeftMotorPosition(){
    return jointLauncherLeftEncoder.getPosition();
  }
  
  public JointLauncherSubsystem() {
    jointLauncherRightMotor.follow(jointLauncherLeftMotor, true);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Right JointLauncher Position",getJointLauncherRightMotorPosition());
    // SmartDashboard.putNumber("Left JointLauncher Position",getJointLauncherLeftMotorPosition());

  }

  public void setMotor(double speed){
    jointLauncherLeftMotor.set(speed);
  }

  public boolean atPosition(double pos){
    if(Math.abs(getJointLauncherLeftMotorPosition()-pos)<1){
      return true;
    }else{return false;}
  }

  public void definePosition(){
    jointLauncherRightEncoder.setPosition(0);
    jointLauncherLeftEncoder.setPosition(0);
  }


}
