// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.JacLib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
    public static final double SPEAKER_Y = Units.inchesToMeters(219.277);

    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
  
    public static final Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), FIELD_WIDTH_METERS);
  
    /** Staging locations for each note */
    public static final class StagingLocations {
      public static final double centerlineX = FIELD_LENGTH_METERS / 2.0;
  
      // need to update
      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);
      // need
      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);
  
      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];
  
      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }
  
      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }
  
    /** Each corner of the speaker * */
    public static final class Speaker {
  
      // corners (blue alliance origin)
      public static final Translation3d topRightSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(238.815),
              Units.inchesToMeters(83.091));
  
      public static final Translation3d topLeftSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(197.765),
              Units.inchesToMeters(83.091));
  
      public static final Translation3d bottomRightSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
      public static final Translation3d bottomLeftSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
  
      /** Center of the speaker opening (blue alliance) */
      public static final Translation3d centerSpeakerOpening =
          bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
    }
  
    public static final class Subwoofer {
      public static final Pose2d ampFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(239.366),
              Rotation2d.fromDegrees(-120));
  
      public static final Pose2d sourceFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(197.466),
              Rotation2d.fromDegrees(120));
  
      public static final Pose2d centerFace =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(218.416),
              Rotation2d.fromDegrees(180));
    }
  
    public static final class Stage {
      public static final Pose2d podiumLeg =
          new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
      public static final Pose2d ampLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(212.425),
              Rotation2d.fromDegrees(-30));
      public static final Pose2d sourceLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(110.837),
              Rotation2d.fromDegrees(30));
  
      public static final Pose2d centerPodiumAmpChain =
          new Pose2d(
              podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(120.0));
      public static final Pose2d centerAmpSourceChain =
          new Pose2d(
              ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
      public static final Pose2d centerSourcePodiumChain =
          new Pose2d(
              sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(240.0));
      public static final Pose2d center =
          new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
      public static final double centerToChainDistance =
          center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
    }
  
    public static final class Amp {
      public static final Translation2d ampTapeTopCorner =
          new Translation2d(Units.inchesToMeters(130.0), Units.inchesToMeters(305.256));
      public static final double ampBottomY = FIELD_WIDTH_METERS - Units.inchesToMeters(17.75);
    }
  
}
