package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private TalonSRX mFrontLeftTalon;
    private TalonSRX mRearLeftTalon;
    private TalonSRX mFrontRightTalon;
    private TalonSRX mRearRightTalon;

    private double m_frontLeftCoeff = 1;
    private double m_rearLeftCoeff = 1;
    private double m_frontRightCoeff = 1;
    private double m_rearRightCoeff = 1;
    private double FrontLeftWheel = 1;
    private double FrontRightWheel = 1;
    private double BackLeftWheel = 1;
    private double BackRightWheel = 1;
    private double mag = 1;
    private double ypower = 1;
    private double xpower = 1;
    private double angle = 1;

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

        double y = ySpeed.getAsDouble();
        double x = xSpeed.getAsDouble();

        mFrontLeftTalon.set(m_driveControlMode, FrontLeftWheel);
        mFrontRightTalon.set(m_driveControlMode, FrontRightWheel);
        mRearLeftTalon.set(m_driveControlMode, BackLeftWheel);
        mRearRightTalon.set(m_driveControlMode, BackRightWheel);

        if (x>1){
          x=1;
        }
        
        if (x<-1){
          x=-1;
        }

        if (y>1){
          y=1;
        }

        if (y<-1){
          y=-1;
        }
        
        mag = Math.sqrt(x*x + y*y);
        angle = Math.atan(y/x);
        ypower = Math.sin(angle-45)*mag;
        xpower = Math.cos(angle-45)*mag;
        xpower = FrontRightWheel = BackLeftWheel;
        ypower = FrontLeftWheel = BackRightWheel;

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

