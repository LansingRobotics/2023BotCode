package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Autonomous extends CommandBase {

  long beginningTime;
  double timeSec;

  public Autonomous() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
    beginningTime = System.currentTimeMillis();
    // Robot.driveTrain.arcadeDrive(RobotMap.NOTHING, RobotMap.NOTHING);
  }

  @Override
  public void execute() {
    timeSec = (double) ((System.currentTimeMillis() - beginningTime) / 1000);

    while (timeSec > 3) {
      
    }
    while (timeSec > 2) {
      
    }
    while (timeSec > 1) {
      
    }
    while (timeSec > 0) {
      
    }

  }

  @Override
  public boolean isFinished() {
    return (timeSec == 15);
  }

  @Override
  public void end(boolean interrupted) {
    // Robot.driveTrain.arcadeDrive(RobotMap.NOTHING, RobotMap.NOTHING);
  }
}
