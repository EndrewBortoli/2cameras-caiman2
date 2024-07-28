package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Robot;

public class PhotonLL extends SubsystemBase {
  //PhotonCamera camera = new PhotonCamera("camara1");
  //PhotonCamera camera = new PhotonCamera("camara1");
  public PhotonCamera FrontRightCamera = new PhotonCamera("Camera_Module_v1");
  public PhotonCamera SideLeftCamera = new PhotonCamera("Camera_Left");

  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;
  private PhotonUtils photon;
  
  public double CAMERA_HEIGHT_METERS = 0.163;
  public double TARGET_HEIGHT_METERS = 1.50;
  public double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-35);

  private double yaw;

  private double pitch;

  private double area;


  private double Id;

  private double xMeters;

  private double yMeters;

  private double zRotation;

  private boolean hasTargets;
  
  public boolean rangeControl;

  public boolean isAligned;

  public double visionControl;


      
    
  /** Creates a new ExampleSubsystem. */
  public PhotonLL() {
   
            photonEstimator =
                new PhotonPoseEstimator(
                        Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, Vision.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    
  }

      public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

        /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
      var estStdDevs = Vision.kSingleTagStdDevs;
      var targets = getLatestResult().getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
          var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = Vision.kMultiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
  }

public double Distance(PhotonPipelineResult result) {
    double range =
      PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(result.getBestTarget().getPitch()));
      return range;
  };

  double correctAngle = -17.6;
  double correctDistance = 3.89;
    public PhotonLL limelight;

  public Double AngleLauncher(){
    var result = camera.getLatestResult();
    // Calcula o ângulo com base na distância atual medida
    double currentDistance = limelight.Distance(result);
    double angle = (correctAngle * correctDistance) / currentDistance; // Proporcionalmente ajustado com base na distância atual
    return angle;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets){
      var target = result.getBestTarget();
  
      // GET DATA:
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      Id = target.getFiducialId();
      Transform3d camToTarget = target.getBestCameraToTarget();
      
      xMeters = camToTarget.getX();
      yMeters = camToTarget.getY();
      zRotation = camToTarget.getRotation().getZ();


    /* 
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Apriltag Id", Id);
*/
      SmartDashboard.putNumber("Y-METERS", yMeters);
        SmartDashboard.putNumber("X-METERS", xMeters);

      SmartDashboard.putNumber("X ANGLE", camToTarget.getRotation().getX());
      SmartDashboard.putNumber("Y ANGLE", -camToTarget.getRotation().getY());
      SmartDashboard.putNumber("Z ANGLE", camToTarget.getRotation().getZ());
      SmartDashboard.putNumber("YAW ANGLE", yaw);
      SmartDashboard.putNumber("Distance", Distance(result));

  
    } 

  }


    private static PhotonLL instance;
    public static  PhotonLL getInstance(){
      if (instance == null){
        instance = new PhotonLL();
      }
      
      return instance;
    }
    
    public boolean hasValueTargets(){
      return hasTargets;
    }

    public double getYaw(){
      return yaw;
    }
  
    public double getYDistance(){
      return yMeters;
    }
    
    public Boolean correctID(int at) {
      if (Id == at) {
        return true;
      } else {
        return false;
      }
    }

    public double getXDistance(){
      return xMeters; 
    }

    public double getZRotation(){
      return zRotation; 
    }

    public double getArea(){
      return area;
    }

    
    public void filterAprilTags(){
      var result = camera.getLatestResult();
    
      
    
      if (result.hasTargets()) {
      //LIST of targets photon vision has
      var targets = result.getTargets();
  
      //checks to see if there is a list of apriltags to check. if no targets are visable, end command
        if (targets.isEmpty()) {
          SmartDashboard.putBoolean("done", true);
        }
  
       var foundTargets = targets.stream().filter(t -> t.getFiducialId() == 4 || t.getFiducialId() == 8)
                        .filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                        .findFirst();
  
        if (foundTargets.isPresent()) {
       var cameraToTarget = foundTargets.get().getBestCameraToTarget();
  }}
    }



    


}
