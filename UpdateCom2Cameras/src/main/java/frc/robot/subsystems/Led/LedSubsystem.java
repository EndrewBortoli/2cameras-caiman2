// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.Led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

         public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
         public static final double RAINBOW_CONFETTI_PALETTE = -0.87;
         public static final double TWINKLE_RAINBOW = -0.55;
         public static final double ORANGE = 0.65;
         public static final double GOLD = 0.67;
         public static final double YELLOW = 0.69;
         public static final double PURPLE = 0.91;
         public static final double BLUE = 0.87;
         public static final double GREEN = 0.77;
         public static final double END_TO_END = 0.45;
         public static final double RAINBOW = 0.35;


   private Double m_color = 0.0;
   private Spark ledController = new Spark(1);

   public LedSubsystem() {
      // SmartDashboard.putNumber("Color", this.m_color);
   }

   public void periodic() {
      this.ledController.set(this.m_color);
   }

   public void setColor(double color) {
      m_color = color;
   }

   }

