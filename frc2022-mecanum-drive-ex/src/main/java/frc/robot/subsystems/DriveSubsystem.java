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
        ySpeed.getAsDouble();
        double fr = 0;
        double rf = 0;
        double x = xSpeed.getAsDouble();
        double y = ySpeed.getAsDouble();
        fr = x;
        rf = fr * -1;
        
        if(y >= 0 && x >= 0){
          //1
          rf += y;
        }
        if(y >= 0 && x <= 0){
          //2
          fr += y;
        }
        if(y <= 0 && x <= 0){
          //3
          rf += y * -1;
        }
        if(y <= 0 && x >= 0){
          //4
          fr += y * -1;
        }
          // if(y >= 0 && x >= 0){
          //   fr = Math.sqrt((y * y)+(x * x)) * -1;
          //   rf = x;
          //   //1
          // }
          // if(y >= 0 && x <= 0){
          //   rf = Math.sqrt((y * y)+(x * x)) * -1;
          //   fr = x;
          //   //2
          // }
          // if(y <= 0 && x <= 0){
          //   fr = Math.sqrt((y * y)+(x * x)) * 1;
          //   rf = x;
          //   //3
          // }
          // if(y <= 0 && x >= 0){
          //   rf = Math.sqrt((y * y)+(x * x)) * 1;
          //   fr = x;
          //   //4
          // }
          if(y <= 0.1 && y >= -0.1 && x <= 0.1 && x >= -0.1){
            rf = 0;
            fr = 0;
          }
                
        mFrontLeftTalon.set(m_driveControlMode, fr);
        mFrontRightTalon.set(m_driveControlMode, rf);
        mRearLeftTalon.set(m_driveControlMode, rf * .75);
        mRearRightTalon.set(m_driveControlMode, fr * .75);
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

