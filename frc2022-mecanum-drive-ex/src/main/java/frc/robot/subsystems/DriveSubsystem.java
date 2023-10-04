package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private TalonSRX mFrontLeftTalon;
    private TalonSRX mRearLeftTalon;
    private TalonSRX mFrontRightTalon;
    private TalonSRX mRearRightTalon;

    private double m_frontLeftCoeff = 1;
    private double m_rearLeftCoeff = 1;
    private double m_frontRightCoeff = 1;
    private double m_rearRightCoeff = 1;

    private ControlMode m_driveControlMode = ControlMode.PercentOutput;

    public DriveSubsystem( TalonSRX mFrontLeftTalon, TalonSRX mRearLeftTalon, TalonSRX mFrontRightTalon, TalonSRX mRearRightTalon) {
      this.mFrontLeftTalon = mFrontLeftTalon;
      this.mRearLeftTalon = mRearLeftTalon;
      this.mFrontRightTalon = mFrontRightTalon;
      this.mRearRightTalon = mRearRightTalon;

      }

   public void drive(DoubleSupplier ySpeed, DoubleSupplier xSpeed)
   {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
        // mRobotDrive.driveCartesian(ySpeed, xSpeed, zRot, 0.0);
       
        double x = xSpeed.getAsDouble();
        double y = ySpeed.getAsDouble();

        
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;

        frontLeftPower = backRightPower;
        frontRightPower = backLeftPower;

        if(Math.abs(x) > Math.abs(y)) {
          mFrontLeftTalon.set(m_driveControlMode, x);
          mFrontRightTalon.set(m_driveControlMode, -x);
          mRearLeftTalon.set(m_driveControlMode, -x);
          mRearRightTalon.set(m_driveControlMode, x);
        } else if(Math.abs(x) < Math.abs(y)) {
          mFrontLeftTalon.set(m_driveControlMode, y);
          mFrontRightTalon.set(m_driveControlMode, y);
          mRearLeftTalon.set(m_driveControlMode, y);
          mRearRightTalon.set(m_driveControlMode, y);
        }
    }


    
      public void setMotorCoeff(
        double frontLeftCoeff,
        double rearLeftCoeff,
        double frontRightCoeff,
        double rearRightCoeff) {
      m_frontLeftCoeff = frontLeftCoeff;
      m_rearLeftCoeff = rearLeftCoeff;
      m_frontRightCoeff = frontRightCoeff;
      m_rearRightCoeff = rearRightCoeff;
    }

      /**
   * Set control mode and velocity scale (opt)
   * 
   * @param controlMode control mode to use setting talon output
   * @param velocityScale velocity for full scale in ticks/100ms
   */
  public void setControlMode(ControlMode controlMode, double velocityScale) {
    m_driveControlMode = controlMode;
  }
}

