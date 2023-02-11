// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class GUI extends SubsystemBase {

  private static SendableChooser<Integer> drivingType = new SendableChooser<>();
  private static SendableChooser<Integer> autonomousCommand = new SendableChooser<>();
  private static SendableChooser<Double> drivingPower = new SendableChooser<>();
  private static SendableChooser<Double> elevatorPower = new SendableChooser<>();
  private static SendableChooser<Double> armLifterPower = new SendableChooser<>();
  private static SendableChooser<Boolean> resettingGyroscope = new SendableChooser<>();
  private static SendableChooser<Double> colorChooser = new SendableChooser<>();

  public void startGui() {
    SmartDashboard.putData("Driving Type", drivingType);
    drivingType.setDefaultOption("Tank Drive", RobotMap.TANK_DRIVE_DRIVING);
    drivingType.addOption("Video Game Driving", RobotMap.VIDEO_GAME_DRIVING);
    drivingType.addOption("Double Tank Drive", RobotMap.DOUBLE_TANK_DRIVING);
    drivingType.addOption("Double Video Game Drive", RobotMap.DOUBLE_VIDEO_GAME_DRIVING);

    SmartDashboard.putData("Color", colorChooser);
    colorChooser.setDefaultOption("Red", RobotMap.RED_COLOR);
    colorChooser.addOption("Blue", RobotMap.BLUE_COLOR);

    SmartDashboard.putData("Autonomous Command", autonomousCommand);
    autonomousCommand.setDefaultOption("Nothing", RobotMap.TANK_DRIVE_DRIVING);
    autonomousCommand.addOption("Autonomous Command", RobotMap.AUTONOMOUS_COMMAND);

    SmartDashboard.putData("Driving Power", drivingPower);
    drivingPower.setDefaultOption("100%", 1.0);
    drivingPower.addOption("90%", 0.9);
    drivingPower.addOption("80%", 0.8);
    drivingPower.addOption("70%", 0.7);
    drivingPower.addOption("60%", 0.6);
    drivingPower.addOption("50%", 0.5);
    drivingPower.addOption("40%", 0.4);
    drivingPower.addOption("30%", 0.3);
    drivingPower.addOption("20%", 0.2);
    drivingPower.addOption("10%", 0.1);
    drivingPower.addOption("Off", 0.0);

    SmartDashboard.putData("Elevator Power", elevatorPower);
    elevatorPower.setDefaultOption("100%", 1.0);
    elevatorPower.addOption("90%", 0.9);
    elevatorPower.addOption("80%", 0.8);
    elevatorPower.addOption("70%", 0.7);
    elevatorPower.addOption("60%", 0.6);
    elevatorPower.addOption("50%", 0.5);
    elevatorPower.addOption("40%", 0.4);
    elevatorPower.addOption("30%", 0.3);
    elevatorPower.addOption("20%", 0.2);
    elevatorPower.addOption("10%", 0.1);
    elevatorPower.addOption("Off", 0.0);

    SmartDashboard.putData("Arm Lifter", armLifterPower);
    armLifterPower.setDefaultOption("100%", 1.0);
    armLifterPower.addOption("90%", 0.9);
    armLifterPower.addOption("80%", 0.8);
    armLifterPower.addOption("70%", 0.7);
    armLifterPower.addOption("60%", 0.6);
    armLifterPower.addOption("50%", 0.5);
    armLifterPower.addOption("40%", 0.4);
    armLifterPower.addOption("30%", 0.3);
    armLifterPower.addOption("20%", 0.2);
    armLifterPower.addOption("10%", 0.1);
    armLifterPower.addOption("Off", 0.0);

    SmartDashboard.putData("Reset Gyroscope", resettingGyroscope);
    resettingGyroscope.setDefaultOption("Off", false);
    resettingGyroscope.addOption("On", true);
  }

  public double getDrivingPower() {
    return drivingPower.getSelected();
  } public double getArmLifterPower() {
    return armLifterPower.getSelected();
  } public double getElevatorPower() {
    return elevatorPower.getSelected();
  } public int getDrivingType() {
    return drivingType.getSelected();
  }

  public int getAutonomousCommand() {

    if (autonomousCommand.getSelected() == null) {
      return 0;
    } else {
      return autonomousCommand.getSelected();
    }
  }

  public boolean getResettingGyroscope() {
    return resettingGyroscope.getSelected();
  }

  public String sayCurrentCommand() {
    switch (getDrivingType()) {
      case RobotMap.TANK_DRIVE_DRIVING:
        return "Tank Driving";
      case RobotMap.VIDEO_GAME_DRIVING:
        return "Video Game Driving";
      case RobotMap.DOUBLE_TANK_DRIVING:
        return "Double Tank Driving";
      case RobotMap.DOUBLE_VIDEO_GAME_DRIVING:
        return "Double Video Game Driving";
      default:
        return "Nothing";
    }
  }

  public void startCamera() {
  }

  @Override
  public void periodic() {
    
  }
}
