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
        if(x != 0){
          if(y > 0 && x < 0){
            rf = xSpeed.getAsDouble();
            //2
          }else if(y < 0 && x > 0){
            rf = xSpeed.getAsDouble();
            //4
          }else if(y > 0 && x > 0){
            fr = xSpeed.getAsDouble();
            //1
          }else if(y < 0 && x < 0){
            fr = xSpeed.getAsDouble();
            //3
          }else if(y == 0){
            rf = xSpeed.getAsDouble() * -1;
            fr = xSpeed.getAsDouble();
            //on the x axis
          }else if(x == 0){
            rf = xSpeed.getAsDouble();
            fr = ySpeed.getAsDouble();
            //on the y axis
          }else{
            //nothing
          }
        }          
        

        mFrontLeftTalon.set(m_driveControlMode, fr);
        mFrontRightTalon.set(m_driveControlMode, rf);
        mRearLeftTalon.set(m_driveControlMode, fr);
        mRearRightTalon.set(m_driveControlMode, rf);
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

