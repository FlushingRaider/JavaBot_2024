package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

import org.littletonrobotics.urcl.URCL;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private ShooterSubsystem shooter;

  @Override
  public void robotInit() {
  
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    SignalLogger.setPath("/U/logs");
    SignalLogger.start();

    DataLogManager.start();
    URCL.start();

    shooter = new ShooterSubsystem(15, 14);
    m_robotContainer = new RobotContainer();
  }

// Periodic is 20ms
  @Override
  public void robotPeriodic() {

    
    // here's how button mapping works
    // see:
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    // aTrigger.onTrue(a_command_you_wanna_run);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-b ased framework to work.

    // CommandScheduler.getInstance().schedule(new SwerveJoystickCmd(
    //     s_Subsystem,
    //     null,
    //     null,
    //     null,
    //     null,
    //     null));

    SmartDashboard.putNumber("upper roller", shooter.getUpperRoller());
    SmartDashboard.putNumber("lower roller", shooter.getLowerRoller());
    
    CommandScheduler.getInstance().schedule(shooter.setPWM(0.15, 0.15));

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic()  {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
