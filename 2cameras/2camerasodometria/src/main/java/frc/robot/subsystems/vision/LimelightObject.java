// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.alignConstraints;

public class LimelightObject extends SubsystemBase {

    private AprilTagFieldLayout m_fieldLayout;

  private static LimelightObject instance;

  private boolean objectIsSeen = true;

  private double x; //April tag x offset

  private double y; //April tag offset

  private double a; //April tag target area

  private double v; //Whether the limelight has any valid targets (0 or 1)

   private double s; //April tag skew

   private double yaw; //April tag yaw

   private int pipelineNumber;

   public boolean rangeControl;

   public boolean isAligned;

   private alignConstraints offsets;

       /**
     * The index of the current pipeline (0 .. 9)
     */
    private NetworkTableEntry pipelineIndex;

    /**
     * Value to set the pipeline to (0 .. 9)
     */
    private NetworkTableEntry setPipeline;

        /**
     * Sets limelight’s LED state
     * 0 = use the LED Mode set in the current pipeline
     * 1 = force off
     * 2 = force blink
     * 3 = force on
     */
    public NetworkTableEntry ledMode;

    /**
     * Sets limelight’s operation mode
     * 0 = Vision Processor
     * 1 = Driver Camera (Increases exposure, disables vision processing)
     */
    public NetworkTableEntry camMode;

        /* AprilTags */



    



  /** Creates a new LimelightObject. */
  public LimelightObject() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Chama as tables da Limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //Pega a entry da pipeline
    table.getEntry("pipeline").setNumber(pipelineNumber);

    //Get Limelight tx
    NetworkTableEntry tx = table.getEntry("tx");
    x = -tx.getDouble(0.0);

    //Get Limelight ty
    NetworkTableEntry ty = table.getEntry("ty");
    y = ty.getDouble(0.0);

    //Get Limelight tv
    NetworkTableEntry tv = table.getEntry("tv");
    v = tv.getDouble(0.0);   

    //Get Limelight ts
    NetworkTableEntry ts = table.getEntry("ts");
    s = ts.getDouble(0.0);


    pipelineIndex = table.getEntry("getpipe");

    setPipeline = table.getEntry("pipeline");

    ledMode = table.getEntry("ledMode");

    camMode = table.getEntry("camMode");




    //Convertendo o parametro de Limelight V de Int para Boolean 

        // SmartDashboard.putNumber("Yaw", yaw);

        // SmartDashboard.putNumber("X", x);
        
        // SmartDashboard.putNumber("S", s);
        
        // SmartDashboard.putNumber("Y", y);
        
        // SmartDashboard.putNumber("A", a);
        
        // SmartDashboard.putBoolean("objetivo?", objectIsSeen);

  }

      /**
     * Void to set the number of the pipeline to use
     * @param alingToAprilTag if it will align to april tag
     * 
     * Se alignToAprilTag is True, o Pipeline é 0, se não é o Pipeline 1
     */
    public void alingToNote(boolean alingToNote){
      if(!alingToNote){
          pipelineNumber = 0;      
      } else {
          pipelineNumber = 0;
      }
  }

  public boolean rangeControl(){
    if(a >= 4 && a <= 8){
      rangeControl = true;
    } else {
      rangeControl = false;
    }
    return rangeControl;
  }

  public boolean isAligned(){
    if(x >= -20 && x <= 20){
      isAligned = true;
    } else {
      isAligned = false;
    }
    return isAligned;
  }

  public double getXLimelight(){
    return x;
  }

  public double getYLimelight(){
    return y;
  }

  public double getALimelight(){
    return v;
  }

  public double getVLimelight(){
    return v;
  }

  public double getYaw(){
    return yaw;
  }

  public boolean getObjectIsSeen(){
    return objectIsSeen;
  
  }
  public boolean hasObject(){
    return objectIsSeen;
  }

  public static LimelightObject getInstance(){
    if(instance == null){
      instance = new LimelightObject();
    }
    return instance;
  }

      /**
     * Sets limelight’s LED state
     * 0 = use the LED Mode set in the current pipeline
     * 1 = force off
     * 2 = force blink
     * 3 = force on*
     * 
     * @param mode LED mode to set
     */
    public void setLedMode(int mode) {
      ledMode.setNumber(mode);
  }

  /**
   * Sets limelight’s cam mode
   * 0 = Vision Processor
   * 1 = Driver Camera (Increases exposure, disables vision processing)
   * 
   * @param mode Cam mode to set
   */
  public void setCamMode(int mode) {
      camMode.setNumber(mode);
  }


  //     private void updateEstimatedGlobalPoses() {
  //   List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

  //   List<Integer> visibleTagIDs = new ArrayList<Integer>();
  //   HashSet<Pose3d> visibleTags = new HashSet<Pose3d>();
  //   List<Pose2d> loggedPoses = new ArrayList<Pose2d>();
  //   for (var camera : m_apriltagCameras) {
  //     var result = camera.getLatestEstimatedPose();
  //     if (result == null) continue;
  //     result.targetsUsed.forEach((photonTrackedTarget) -> {
  //       if (photonTrackedTarget.getFiducialId() == -1) return;
  //       visibleTagIDs.add(photonTrackedTarget.getFiducialId());
  //       visibleTags.add(m_fieldLayout.getTagPose(photonTrackedTarget.getFiducialId()).get());
  //     });
  //     estimatedPoses.add(result);
  //     loggedPoses.add(result.estimatedPose.toPose2d());
  //   }

  //   // Log visible tags and estimated poses
  //   Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY, visibleTags.toArray(new Pose3d[0]));
  //   Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY, loggedPoses.toArray(new Pose2d[0]));

  //   m_visibleTagIDs.set(visibleTagIDs);
  //   m_estimatedRobotPoses.set(estimatedPoses);
  // }
  
    public Boolean objectIsSeen() {
    if (v > 0){
      objectIsSeen = true;
    } else {
      objectIsSeen = false;
    }
    return objectIsSeen;
  }

}
