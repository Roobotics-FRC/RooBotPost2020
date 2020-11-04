package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Drives a distance in the specified direction. Field/bot-oriented driving is determined by the
 * drivetrain's current drive mode (own-ship- or field-up).
 */
public class DriveDistanceAuton extends CommandBase {
    private static final RobotMap.PID DRIVE_GAINS = new RobotMap.PID(0, 0.01, 0, 0);
    private static final double PID_OUTPUT_THRESHOLD = 0.2;

    private PIDController distanceController;
    private PIDController rotationController;

    private Drivetrain drivetrain;

    private double distance;
    private double speed;
    private double angle;

    private Translation2d originalPosition;

    private boolean finished = false;

    /**
     * Constructs a new distance-driving auton.
     * @param distance the distance to drive in inches.
     * @param angle the angle at which to drive.
     */
    public DriveDistanceAuton(double distance, double speed, double angle) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());

        this.distance = distance;
        this.speed = speed;
        this.angle = angle;

        this.distanceController = new PIDController(DRIVE_GAINS.kP, DRIVE_GAINS.kI, DRIVE_GAINS.kD);
        this.rotationController = new PIDController(RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kP,
                RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kI,
                RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kD);
        //TODO: this.rotationController.setOutputRange(-1, 1);
    }

    @Override
    public void initialize() {
        super.initialize();
        originalPosition = drivetrain.getPose().getTranslation();
        this.finished = false;
        this.distanceController.setSetpoint(Units.inchesToMeters(distance));
        this.distanceController.reset();
        this.rotationController.setSetpoint(rotationPIDInput());
        this.rotationController.reset();
    }

    private double distancePIDInput() {
        return drivetrain.getPose().getTranslation().getDistance(originalPosition);
    }

    private double rotationPIDInput() {
        return drivetrain.getPose().getRotation().getDegrees();
    }

    @Override
    public void execute() {
        useOutput(distanceController.calculate(distancePIDInput()),
                rotationController.calculate(rotationPIDInput()));
    }

    private void useOutput(double distancePIDOutput, double rotationPIDOutput) {
        SmartDashboard.putNumber("swerve/auton_output", distancePIDOutput);
        if (Math.abs(distancePIDOutput) <= PID_OUTPUT_THRESHOLD) {
            this.finished = true;
            return;
        }
        double x = Math.cos(Math.toRadians(this.angle)) * speed;
        double y = Math.sin(Math.toRadians(this.angle)) * speed;
        this.drivetrain.drive(rotationPIDOutput, x, y);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.stop();
    }
}