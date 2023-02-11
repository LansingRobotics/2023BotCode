package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Commands.SwitchDrive;

public class DriveTrain extends SubsystemBase {

  private static final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  // private static final AnalogPotentiometer ultraSonicSensor = new AnalogPotentiometer(0, 0, 9600);


  private static final Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private static final DoubleSolenoid solenoidOne = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.PNEUMATIC_FORWARD, RobotMap.PNEUMATIC_REVERSE);
  private static final Spark ledLights = new Spark(RobotMap.LED_LIGHTS);

  private static final CANSparkMax frontRightMController = new CANSparkMax(RobotMap.RIGHT_FRONT_DRIVE, MotorType.kBrushless);
  private static final CANSparkMax frontLeftMController = new CANSparkMax(RobotMap.LEFT_FRONT_DRIVE, MotorType.kBrushless);
  private static final CANSparkMax rearRightMController = new CANSparkMax(RobotMap.RIGHT_REAR_DRIVE, MotorType.kBrushless);
  private static final CANSparkMax rearLeftMController = new CANSparkMax(RobotMap.LEFT_REAR_DRIVE, MotorType.kBrushless);
  private static final CANSparkMax armLifterMController = new CANSparkMax(RobotMap.ARM_LIFTER, MotorType.kBrushless);
  private static final CANSparkMax thriftyElevatorMController = new CANSparkMax(RobotMap.ELEVATOR_MOTOR, MotorType.kBrushless);
  
  private static final RelativeEncoder frontRightMEncoder = frontRightMController.getEncoder();
  private static final RelativeEncoder frontLeftMEncoder = frontLeftMController.getEncoder();
  private static final RelativeEncoder rearRightMEncoder = rearRightMController.getEncoder();
  private static final RelativeEncoder rearLeftMEncoder = rearLeftMController.getEncoder();
  private static final RelativeEncoder armLifterMEncoder = armLifterMController.getEncoder();
  private static final RelativeEncoder thriftyElevatorMEncoder = thriftyElevatorMController.getEncoder();

  public double realAngle;
  public double batteryVoltage;


  public void startDriveTrain() {
    armLifterMController.restoreFactoryDefaults();
    frontRightMController.restoreFactoryDefaults();
    frontLeftMController.restoreFactoryDefaults();
    rearRightMController.restoreFactoryDefaults();
    rearLeftMController.restoreFactoryDefaults();
    thriftyElevatorMController.restoreFactoryDefaults();
    pcmCompressor.enableDigital();
    solenoidOne.set(Value.kForward);
  }

  public void initDefaultCommand() {
    setDefaultCommand(new SwitchDrive());
  }

        public double getArmLifterVelocity() {
          return armLifterMEncoder.getVelocity();
        } public double getArmLifterPosition() {
          return armLifterMEncoder.getPosition();
        } public double getArmLifterTemp() {
          return armLifterMController.getMotorTemperature();
        } public double getArmLifterOutputAmps() {
          return armLifterMController.getOutputCurrent();
        } public double getArmLifterOutputVolts() {
          return armLifterMController.getAppliedOutput();
        }

        public double getFrontRightVelocity() {
          return frontRightMEncoder.getVelocity();
        } public double getFrontRightPosition() {
          return frontRightMEncoder.getPosition();
        } public double getFrontRightTemp() {
          return frontRightMController.getMotorTemperature();
        } public double getFrontRightOutputAmps() {
          return frontRightMController.getOutputCurrent();
        } public double getFrontRightOutputVolts() {
          return frontRightMController.getAppliedOutput();
        }

        public double getFrontLeftVelocity() {
          return frontLeftMEncoder.getVelocity();
        } public double getFrontLeftPosition() {
          return frontLeftMEncoder.getPosition();
        } public double getFrontLeftTemp() {
          return frontLeftMController.getMotorTemperature();
        } public double getFrontLeftOutputAmps() {
          return frontLeftMController.getOutputCurrent();
        } public double getFrontLeftOutputVolts() {
          return frontLeftMController.getAppliedOutput();
        }

        public double getRearRightVelocity() {
          return rearRightMEncoder.getVelocity();
        } public double getRearRightPosition() {
          return rearRightMEncoder.getPosition();
        } public double getRearRightTemp() {
          return rearRightMController.getMotorTemperature();
        } public double getRearRightOutputAmps() {
          return rearRightMController.getOutputCurrent();
        } public double getRearRightOutputVolts() {
          return rearRightMController.getAppliedOutput();
        }

        public double getRearLeftVelocity() {
          return rearLeftMEncoder.getVelocity();
        } public double getRearLeftPosition() {
          return rearLeftMEncoder.getPosition();
        } public double getRearLeftTemp() {
          return rearLeftMController.getMotorTemperature();
        } public double getRearLeftOutputAmps() {
          return rearLeftMController.getOutputCurrent();
        } public double getRearLeftOutputVolts() {
          return rearLeftMController.getAppliedOutput();
        }

        public double getElevatorVelocity() {
          return thriftyElevatorMEncoder.getVelocity();
        } public double getElevatorPosition() {
          return thriftyElevatorMEncoder.getPosition();
        } public double getElevatorTemp() {
          return thriftyElevatorMController.getMotorTemperature();
        } public double getElevatorOutputAmps() {
          return thriftyElevatorMController.getOutputCurrent();
        } public double getElevatorOutputVolts() {
          return thriftyElevatorMController.getAppliedOutput();
        }  

  
  public void arcadeDrive(double leftSpeed, double rightSpeed) {
    frontLeftMController.set(leftSpeed * Robot.gui.getDrivingPower());
    rearLeftMController.set(leftSpeed * Robot.gui.getDrivingPower());

    frontRightMController.set(-rightSpeed * Robot.gui.getDrivingPower());
    rearRightMController.set(-rightSpeed * Robot.gui.getDrivingPower());
  }

  public void toggleSolenoidOne(boolean forward, boolean reverse) {
    if (forward) {
      solenoidOne.set(Value.kForward);
    }
    if (reverse) {
      solenoidOne.set(Value.kReverse);
    }
  }

  public boolean getIsTheCompressorOn() {
    return !pcmCompressor.getPressureSwitchValue();
  }

  public void setColorOfLED(double color) {
    ledLights.set(color);
  }

  // public double getUltraSonicDistance() {
  //   return ultraSonicSensor.get();
  // }

  public void liftTheArm(double power) {
    armLifterMController.set(power * Robot.gui.getArmLifterPower());
  }

  public void liftTheElevator(double power) {
    thriftyElevatorMController.set(power * Robot.gui.getElevatorPower());
  }

  public double getRate() {
    return Math.round(gyroscope.getRate());
  }

  public double getAngle() {
    realAngle = Math.round(gyroscope.getAngle()) - (360 * Math.floor(Math.round(gyroscope.getAngle()) / 360));
    if (realAngle < RobotMap.NOTHING) {
      return (realAngle + 360);
    } else {
      return realAngle;
    }
  }

  public void calibrate() {
    gyroscope.calibrate();
  }

  public void gyroReset() {
    gyroscope.reset();
  }

  public double getBatteryVoltage() {
    batteryVoltage = RobotController.getBatteryVoltage();
    return batteryVoltage;
  }

  public boolean doWeHavePower() {
    if (batteryVoltage <= 8) {
      return false;
    } else {
      return true;
    }
  }

}
