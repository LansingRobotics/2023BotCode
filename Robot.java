package frc.robot;

import java.text.DecimalFormat;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import frc.robot.Commands.Autonomous;
import frc.robot.Commands.SwitchDrive;
import frc.robot.Subsystems.Controllers;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.GUI;


public class Robot extends TimedRobot {
  
  public static DriveTrain driveTrain = new DriveTrain();
  public static final DecimalFormat twoDecimalPlaceFormat = new DecimalFormat("###.##");
  public static final DecimalFormat wholeNumberFormat = new DecimalFormat("###");
  public static GUI gui = new GUI();
  public static Controllers controller1 = new Controllers(RobotMap.DRIVER_CONTROLLER1);
  public static Controllers controller2 = new Controllers(RobotMap.DRIVER_CONTROLLER2);
  private static SwitchDrive switchDrive = new SwitchDrive();
  private static Autonomous auton = new Autonomous();
  private static Thread m_visionThread;


  @Override
  public void robotInit() {
    gui.startGui();
    driveTrain.calibrate();
    driveTrain.startDriveTrain();
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, switchDrive);
    m_visionThread = new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(10, 8);
      CvSink cvSink = CameraServer.getVideo();
      CvSource outputSteam = CameraServer.putVideo("Robot Camera", 10, 8);
      Mat mat = new Mat();
      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(mat) == 0) {
          outputSteam.notifyError(cvSink.getError());
          continue;
        }
        outputSteam.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putString("Battery Voltage", twoDecimalPlaceFormat.format(driveTrain.getBatteryVoltage()) + " Volts");
    SmartDashboard.putString("Currently Doing", gui.getDrivingType() + "");
    SmartDashboard.putBoolean("Power Avaliability", driveTrain.doWeHavePower());
    SmartDashboard.putString("Gyroscope Angle", driveTrain.getAngle() + " Degrees");
    SmartDashboard.putString("Gyroscope Rate", driveTrain.getRate() + " Degrees / Sec");
    // SmartDashboard.putNumber("Rear Left Temp", Robot.driveTrain.getRearLeftTemp());
    // SmartDashboard.putNumber("Rear Right Temp", Robot.driveTrain.getRearRightTemp());
    // SmartDashboard.putNumber("Front Left Temp", Robot.driveTrain.getFrontLeftTemp());
    // SmartDashboard.putNumber("Front Right Temp", driveTrain.getFrontRightTemp());
    // SmartDashboard.putNumber("Arm Temp", driveTrain.getArmLifterTemp());
    // SmartDashboard.putNumber("Elevator Temp", driveTrain.getElevatorPosition());
    // SmartDashboard.putNumber("UltraSonic Distance", driveTrain.getUltraSonicDistance());
    SmartDashboard.putBoolean("Compressor On", driveTrain.getIsTheCompressorOn());

    CommandScheduler.getInstance().run();

    if (DriverStation.getAlliance() == Alliance.Blue) {
      driveTrain.setColorOfLED(RobotMap.BLUE_COLOR);
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      driveTrain.setColorOfLED(RobotMap.RED_COLOR);
    }

    if (gui.getResettingGyroscope()) {
      driveTrain.gyroReset();
    }

  }

  @Override
  public void autonomousInit() {
    auton.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    switchDrive.schedule();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

}
