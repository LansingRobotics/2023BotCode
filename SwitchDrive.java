package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwitchDrive extends CommandBase {
  static double rightTurnValue;
  static double leftTurnValue;

  public SwitchDrive() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
    Robot.driveTrain.arcadeDrive(RobotMap.NOTHING, RobotMap.NOTHING);
  }

  @Override
  public void execute() {
    switch (Robot.gui.getDrivingType()) {
      case RobotMap.TANK_DRIVE_DRIVING:
        singleTankDriveExecute();
        break;
      case RobotMap.DOUBLE_TANK_DRIVING:
        doubleTankDriveExecute();
        break;
      case RobotMap.VIDEO_GAME_DRIVING:
        singleVideoGameDriveExecute();
        break;
      case RobotMap.DOUBLE_VIDEO_GAME_DRIVING:
        doubleVideoGameDriveExecute();
        break;
      default:
        singleTankDriveExecute();
        break;
    }
    SmartDashboard.putString("Drive Mode", Robot.gui.sayCurrentCommand());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.arcadeDrive(RobotMap.NOTHING, RobotMap.NOTHING);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static void singleTankDriveExecute() {
    Robot.driveTrain.arcadeDrive(Robot.controller1.getDriverRawAxis(RobotMap.LEFT_STICK_Y), Robot.controller1.getDriverRawAxis(RobotMap.RIGHT_STICK_Y));

    Robot.driveTrain.liftTheArm(Robot.controller1.getDifferenceInTriggers());

    Robot.driveTrain.liftTheElevator(Robot.controller1.getDifferenceInBumpers());

    Robot.driveTrain.toggleSolenoidOne(Robot.controller1.getIsButtonPressed(RobotMap.X_BUTTON), Robot.controller1.getIsButtonPressed(RobotMap.A_BUTTON));
  }

  private static void singleVideoGameDriveExecute() {
    double turnValue = Robot.controller1.getDriverRawAxis(RobotMap.LEFT_STICK_X);

    if (turnValue >= 0) {
      rightTurnValue = -turnValue;
    } else {
      rightTurnValue = 0;
    }
    if (turnValue < 0) {
      leftTurnValue = turnValue;
    } else {
      leftTurnValue = 0;
    }

    Robot.driveTrain.arcadeDrive(
        ((Robot.controller1.getDifferenceInTriggers() - leftTurnValue)),
        ((Robot.controller1.getDifferenceInTriggers() - rightTurnValue)));

    Robot.driveTrain.liftTheArm(Robot.controller1.getDriverRawAxis(RobotMap.RIGHT_STICK_Y));

    Robot.driveTrain.liftTheElevator(Robot.controller1.getDifferenceInBumpers());

    Robot.driveTrain.toggleSolenoidOne(Robot.controller1.getIsButtonPressed(RobotMap.X_BUTTON), Robot.controller1.getIsButtonPressed(RobotMap.A_BUTTON));

  }

  private static void doubleTankDriveExecute() {
    Robot.driveTrain.arcadeDrive(Robot.controller1.getDriverRawAxis(RobotMap.LEFT_STICK_Y), Robot.controller1.getDriverRawAxis(RobotMap.RIGHT_STICK_Y));

    Robot.driveTrain.liftTheArm(Robot.controller2.getDriverRawAxis(RobotMap.RIGHT_STICK_Y));

    Robot.driveTrain.liftTheElevator(Robot.controller2.getDriverRawAxis(RobotMap.LEFT_STICK_Y));

    Robot.driveTrain.toggleSolenoidOne(Robot.controller2.getIsButtonPressed(RobotMap.X_BUTTON), Robot.controller2.getIsButtonPressed(RobotMap.A_BUTTON));
  }

  private static void doubleVideoGameDriveExecute() {
    double turnValue = Robot.controller1.getDriverRawAxis(RobotMap.LEFT_STICK_X);

    if (turnValue >= 0) {
      rightTurnValue = turnValue;
    } else {
      rightTurnValue = 0;
    }
    if (turnValue < 0) {
      leftTurnValue = -turnValue;
    } else {
      leftTurnValue = 0;
    }

    Robot.driveTrain.arcadeDrive(((Robot.controller1.getDifferenceInTriggers() - leftTurnValue)), ((Robot.controller1.getDifferenceInTriggers() - rightTurnValue)));

    Robot.driveTrain.liftTheArm(Robot.controller2.getDriverRawAxis(RobotMap.RIGHT_STICK_Y));

    Robot.driveTrain.liftTheElevator(Robot.controller2.getDriverRawAxis(RobotMap.LEFT_STICK_Y));

    Robot.driveTrain.toggleSolenoidOne(Robot.controller2.getIsButtonPressed(RobotMap.X_BUTTON), Robot.controller2.getIsButtonPressed(RobotMap.A_BUTTON));

  }

}