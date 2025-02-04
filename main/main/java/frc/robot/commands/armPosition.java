package frc.robot.commands;
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class armPosition extends Command {

  private final Arm arm;
  private boolean reached;
  private double position;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public armPosition(Arm arm, double position) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    reached = false;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //arm.setPosition(40);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arm.getPosition()>=position){
      System.out.println("NEGATIVE");
      System.out.println(arm.getPosition());
      arm.setSpeed(-1);
      if(position-0.02 <= arm.getPosition() && arm.getPosition() <= position+0.02){
        reached = true;
        System.out.println("REACHED1");
      } 
    }
    else if(arm.getPosition()<=position){
      arm.setSpeed(1);
      System.out.println("POSITIVE");
      System.out.println(arm.getPosition());
      if(position-0.02 <= arm.getPosition() && arm.getPosition() <= position+0.02){
        reached = true;
        System.out.println("REACHED2");
      } 

    } 
    else{
      System.out.println("REACHED3");
      reached = true;
    } 
    
    if(arm.getPosition() >= 0.39){
      reached = true;
    }

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
    System.out.println("REACHED4");
    reached = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reached;
  }
}

