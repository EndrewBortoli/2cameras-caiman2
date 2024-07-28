// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.JointLauncher.JointLauncherSubsystem;


/** Add your docs here. */
public class LimelightLaunching extends SubsystemBase {

    // Criando uma instância de MeasuringDistance

    public PhotonLL photonll;
    public double launchAngle;

    private static JointLauncherSubsystem jointLauncherSubsystem = new JointLauncherSubsystem();

    double launcherJointAngle = jointLauncherSubsystem.getJointLauncherLeftMotorPosition();

    public double definingAngle(){
        this.photonll = PhotonLL.getInstance();

    int speakerOpeningY1 = 78;
    int speakerOpeningX1 = 0;

    double speakerOpeningY2 = 18.11;
    double speakerOpeningX2 = 82.875;
    
    double theta1 = ((speakerOpeningY1 - 85)/(photonll.getXDistance() - speakerOpeningX1))*180/Math.PI;

    double theta2 = ((speakerOpeningY2 - 85)/(photonll.getXDistance() - speakerOpeningX2))*180/Math.PI;

    double phi = (theta2 - theta1);

    double launchAngle = (theta1 + theta2)/2;

    return launchAngle;
    }


    public boolean enableShooting(){
        
        if (launcherJointAngle == definingAngle()){
            return enableShooting() == true;
        } else {
            return false;
        }
    }



}
