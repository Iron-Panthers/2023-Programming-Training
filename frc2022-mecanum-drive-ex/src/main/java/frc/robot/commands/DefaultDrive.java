package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    
    // Your suppliers
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    // Drivebase subsystem
    private DriveSubsystem mDrive;

    public DefaultDrive (DoubleSupplier xSupplier, DoubleSupplier ySupplier, DriveSubsystem mDrive) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.mDrive = mDrive;
        addRequirements(mDrive);
    }


    @Override
    public void initialize() {
      // Anything that needs to be reset between commands
      //m_drive.resetEncoders();
    }
  
    @Override
    public void execute() {

        mDrive.drive(ySupplier, xSupplier);

        // get values from suppliers
        // double y = ySupplier.getAsDouble();
        // double x = xSupplier.getAsDouble();

        // Use your drive method
       
    }
  
    @Override
    public void end(boolean interrupted) {

      // Set your drive method to not move
    }
  
    // @Override
    // public boolean isFinished() {
    // //   return mTimer.get() >= mTimeS;
    // }


}
