package frc.robot.subsystems.Launcher;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    private final CANSparkFlex launcherMotor = new CANSparkFlex((int) LauncherConstants.kLauncherMotorId, MotorType.kBrushless);
    private final CANSparkMax triggerMotor = new CANSparkMax((int) LauncherConstants.kTriggerMotorId, MotorType.kBrushless);

    public boolean noteIntaken;
    public double motorCurrent = launcherMotor.getOutputCurrent();




    public boolean noteIntaken(){
        if (motorCurrent >= 40){
            noteIntaken = true;
        } else {
            noteIntaken = false;
        }
        return noteIntaken;
    }
    public LauncherSubsystem(){
        
    launcherMotor.setIdleMode(IdleMode.kCoast);
    triggerMotor.setIdleMode(IdleMode.kBrake);
    
    launcherMotor.setSmartCurrentLimit(80);
    triggerMotor.setSmartCurrentLimit(10);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Corrente do Motor NEO (A)", launcherMotor.getOutputCurrent());
    }

    public void setLauncher(String state){

        if(state == "Launch") {
            launcherMotor.set(LauncherConstants.kLauncherMotorVelocity);
            triggerMotor.set(LauncherConstants.kLauncherMotorVelocity);
        }

        if(state == "Trigger"){
            triggerMotor.set(-LauncherConstants.kTriggerMotorVelocity);
        }

        if(state == "Trigger Amp"){
            triggerMotor.set(LauncherConstants.kTriggerMotorVelocity);
        }

        if(state == "Intake") {
            launcherMotor.set(-LauncherConstants.kLauncherMotorVelocity);
        }

        if(state == "Static") {
            launcherMotor.set(0);
            triggerMotor.set(0);
        }

    }
    
}