// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.Led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
   private Double m_color = 0.0;
   private Spark ledController = new Spark(1);

   public LedSubsystem() {
      // SmartDashboard.putNumber("Color", this.m_color);
   }

   public void periodic() {
      this.ledController.set(this.m_color);
   }

   public void setColor(String color) {
      if (color == "green") {
         this.m_color = 0.77;
      }

      if (color == "red") {
         this.m_color = 0.61;
      }

      if (color == "blue") {
         this.m_color = 0.87;
      }

      if (color == "purple") {
         this.m_color = 0.91;
      }

      if (color == "yellow") {
         this.m_color = 0.69;
      }

      if (color == "orange") {
         this.m_color = 0.65;
      }
      
      if (color == "rainbow") {
         this.m_color = 0.35;
      }

      if (color == "orange") {
         this.m_color = 0.65;
      }

      if (color == "orange") {
         this.m_color = 0.65;
      }

      

   }
}
