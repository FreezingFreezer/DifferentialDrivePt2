package robot;

import static edu.wpi.first.units.Units.Seconds;
import static robot.Constants.PERIOD;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.CommandRobot;
import lib.FaultLogger;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;
import robot.Ports.OI;
import robot.drive.Drive;



// Reference: https://github.com/AnkitKumar5250/SummerInstitute2024Team1/blob/main/src/main/java/frc/robot/Robot.java
public class Robot extends CommandRobot implements Logged {
  @Log.NT
  private final Drive drive = new Drive();

  private final CommandXboxController operator = new CommandXboxController(Ports.OI.OPERATOR);

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance()
      .schedule(drive.drive(new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return operator.getLeftX();
        }
      }, new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return operator.getRightX();
        }
      })); 
      DataLogManager.start();
      Monologue.setupMonologue(this, "/Robot", false, true);
      addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
      addPeriodic(FaultLogger::update, 2);
  }
  
  @Override
  public void teleopInit() {
    // Cancels all autonomous commands at the beggining of telop
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationInit() {
    // Adds field visualizer to dashboard
    
  }

  @Override
  public void simulationPeriodic() {

  }
}
