package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hooks extends SubsystemBase{
    private final PWM leftHook;
    private final PWM rightHook;

    public Hooks(){
        leftHook = new PWM(1);
        rightHook = new PWM(2);
    }

    public void rotate(double angle){
        rightHook.setPosition(angle);
        leftHook.setPosition(-angle);
        
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
