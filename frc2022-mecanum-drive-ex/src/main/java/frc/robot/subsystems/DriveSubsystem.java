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
        double flRr = (x + y) / (Math.sqrt(2));
        double frRl = (x * -1 + y) / (Math.sqrt(2));
        //using the distance from a point and a line formula( d = ∣ a x  + b y  + c∣ / sqrt(a2 + b2) ), we can make
        // two new axises, which we are going to call xAxis2 and yAxis2. These axises are rotated 45 degrees. Knowing that, we*
        // can draw two lines for our new axises; the slope of xAxis2 is x+y=0, and the slope of yAxis2 is -x+y=0. With these lines,
        // we can implement it into the distance of a point and a line formula; xAxis2: |x+y|/sqrt(a2 + b2) because ax + by + c represents 
        // the slope of the line. From this, we can conclude that a = 1 and b = 1 and c = 0, so implementing that into our equation, we get
        // |x+y|/sqrt(1^2 + 1^2), simplifying, we get |x+y|/sqrt(2). Doing the same thing for the yAxis2, we get |-x+y|/sqrt(2). 
        // Now, because we need to find the negative distance too(because we are making a new graph), we delete the absolute value, gettng us
        // xAxis2: |x+y|/sqrt(2). Doing the same thing for the yAxis2, we get |-x+y|/sqrt(2). This is how I got my equation.
        mFrontLeftTalon.set(m_driveControlMode, flRr);
        mFrontRightTalon.set(m_driveControlMode, frRl);
        mRearLeftTalon.set(m_driveControlMode, frRl * 0.75);
        mRearLeftTalon.set(m_driveControlMode, flRr * 0.75);
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

