/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

 package frc.lib.util;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;


 public class alignConstraints {
     
     public double driveOffset;
     public double strafeOffset;
     public double rotationOffset;
     public ProfiledPIDController drivePID, strafePID, rotationPID;

 
     public alignConstraints(
        double driveValue,
        double strafeValue,
        double rotationOffset,
        PIDController drivePID2,
        PIDController strafePID2,
        PIDController rotationPID2     
     ){
         this.driveOffset = driveValue;
         this.strafeOffset = strafeValue;
         this.rotationOffset = rotationOffset;
        //  this.drivePID = drivePID2;
        //  this.strafePID = strafePID2;
        //  this.rotationPID = rotationPID2;    
 
     }
 }
 