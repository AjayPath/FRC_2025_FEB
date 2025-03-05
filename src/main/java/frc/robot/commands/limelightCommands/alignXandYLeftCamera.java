// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelightCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class alignXandYLeftCamera extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController strafePIDx; 
  private PIDController drivePIDArea; 


  private boolean endCommand; 
  private int setPipelineNumber; 

  private double measuredValuex; 
  private double measuredValuey; 

  private double strafeSpeedx; 
  private double driveSpeedy; 


  private double targetValuex; 
  private double targetValuey; 

  private double xTolreance; 
  private double yTolreance; 

  private boolean inRangeX; 
  private boolean inRangeY;  


  private int missedCounter; 


  /** Creates a new alignXandYLeftCamera. */
  public alignXandYLeftCamera(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end, double targetX, double targetY, double toleranceX, double toleranceY) {
    // Use addRequirements() here to declare subsystem dependencies.
        
    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.drivePIDArea = new PIDController(0.05, 0, 0.01); 
    this.strafePIDx = new PIDController(0.05, 0, 0.01); 
    this.endCommand = end; 
    this.setPipelineNumber = pipeline; 
    
    this.targetValuex = targetX; 
    this.targetValuey = targetY; 
    this.xTolreance = toleranceX; 
    this.yTolreance = toleranceY; 

    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePIDArea.reset();
    strafePIDx.reset(); 
    VISION_SUBSYSTEM.setLeftPipeline(setPipelineNumber);
    inRangeX = false;
    inRangeY = false;  
    missedCounter = 0; 

    targetValuex = correctedTx(targetValuex); 
    xTolreance = correctedTx(xTolreance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(VISION_SUBSYSTEM.limelightLeftTargetSeen()){

        measuredValuex = correctedTx(VISION_SUBSYSTEM.getLeftTx()); 

        if (Math.abs(targetValuex - measuredValuex) <= xTolreance){ 
          strafeSpeedx = 0; 
          inRangeX = true;
        } 
        
        else{
          strafeSpeedx = strafePIDx.calculate(measuredValuex,targetValuex);
          inRangeX = false;      
        }
     
        if (Math.abs(targetValuey - measuredValuey) <= yTolreance){ 
          driveSpeedy = 0; 
          inRangeY = true;
        } 
  
        else{
          driveSpeedy = drivePIDArea.calculate(measuredValuey, targetValuey);
          inRangeY = false; 
       
        }

        driveSpeedy = smoothSpeedLimit(driveSpeedy, 0.05, 0.5); 
        strafeSpeedx = smoothSpeedLimit(strafeSpeedx, 0.05, 0.5); 
        
        // if(driveSpeedy > 0.15){
        //   driveSpeedy = 0.15; 
        // }else if(driveSpeedy < -0.15){
        //   driveSpeedy = -0.15; 
        // }


        // if(strafeSpeedx > 0.10){
        //   strafeSpeedx = 0.10; 
        // }else if(strafeSpeedx < -0.10){
        //   strafeSpeedx = -0.10; 
        // }
        
    }else{
      strafeSpeedx = 0; 
      driveSpeedy = 0; 
      missedCounter += 1; 
    }

    SmartDashboard.putNumber("align speed", strafeSpeedx); 
    DRIVE_SUBSYSTEM.drive(-driveSpeedy, -strafeSpeedx, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSpeedy = 0; 
    strafeSpeedx = 0; 
  }

  @Override
  public boolean isFinished() {
    if(endCommand){
      return true; 
    }
    
    else{
      return false; 
    }

  }

  public double correctedTx(double tx){    

    double rollRadians = Math.toRadians(30); 
    double txTrue = tx * Math.cos(rollRadians); 

    return txTrue; 
  }

  
  private double smoothSpeedLimit(double speed, double maxSpeed, double rampFactor) {
    return speed * rampFactor + (1 - rampFactor) * maxSpeed * Math.signum(speed);
  }
}

