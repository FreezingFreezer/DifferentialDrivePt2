package main.java.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;

public class Drive extends SubsystemBase {  
    private final DifferentialDrivetrainSim driveSim;
    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
    private final DifferentialDriveOdometry odometry;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
    private final PIDController leftPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
    private final PIDController rightPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
    @Log.NT 
    private final Field2d field2d = new Field2d();
    public Drive() {
        for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
            spark.restoreFactoryDefaults(); //resets motors
            spark.setIdleMode(IdleMode.kBrake); //stops motors

        }
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

        rightFollower.follow(rightLeader); //just makes it so that they follow the same direction and speed
        leftFollower.follow(leftLeader); 
    
        leftLeader.setInverted(true); //since its at a diff direction just reverses values to reverse rotation

        leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);

        leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    
        gyro.reset();

        double leftVoltage = leftPID + leftFeedforward;
        double rightVoltage = rightPID + rightFeedforward;

        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);
        
        driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
    }
    private void drive(double leftSpeed, double rightSpeed) { //gives the speeds to the motors
        leftLeader.set(leftSpeed); 
        rightLeader.set(rightSpeed);

        final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
	    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
	
        final double leftFeedforward = feedforward.calculate(realLeftSpeed);
        final double rightFeedforward = feedforward.calculate(realRightSpeed);

        final double leftPID = 
        leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
        final double rightPID = 
        rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
        
        driveSim.setInputs(leftVoltage, rightVoltage);
    }
    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }
    
    private void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

    public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  

    @Override 
  public void periodic() {
    updateOdometry(gyro.getRotation2d());
    updateOdometry(Robot.isReal() ? gyro.getRotation2d() : driveSim.getHeading());
    field2d.setRobotPose(pose());
} 

    @Override
  public void simulationPeriodic() {
    // sim.update() tells the simulation how much time has passed
    driveSim.update(Constants.PERIOD.in(Seconds));
    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
  }
  
}
   
