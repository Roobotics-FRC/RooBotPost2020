package frc.team4373.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.team4373.robot.commands.camera.CameraCommand;
import frc.team4373.robot.commands.climber.ClimberCommand;
import frc.team4373.robot.commands.drivetrain.DrivetrainCommand;
import frc.team4373.robot.commands.intake.IntakeCommand;
import frc.team4373.robot.commands.shooter.ShooterCommand;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.*;

import java.util.List;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        Camera.getInstance().setDefaultCommand(new CameraCommand());
        Climber.getInstance().setDefaultCommand(new ClimberCommand());
        Intake.getInstance().setDefaultCommand(new IntakeCommand());
        Shooter.getInstance().setDefaultCommand(new ShooterCommand());
        Drivetrain.getInstance().setDefaultCommand(new DrivetrainCommand());
        // WheelSpinner.getInstance().setDefaultCommand(new WheelSpinnerCommand());
    }

    @Override
    public void autonomousInit() {
        TrajectoryConfig config = new TrajectoryConfig(RobotMap.MAX_SPEED_METERS_PER_SECOND,
                RobotMap.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Drivetrain.getInstance().kinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d()),
                config
        );

        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
                RobotMap.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                () -> Drivetrain.getInstance().getPose(),
                Drivetrain.getInstance().kinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0), thetaController,
                Drivetrain.getInstance()::setModuleStates,
                Drivetrain.getInstance()
        );

        swerveControllerCommand.schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("shooter/encoder_vel", Shooter.getInstance().getVelocity());
        SmartDashboard.putNumber("shooter/motor1_output",
                Shooter.getInstance().getMotor1PercentOutput());
        SmartDashboard.putNumber("shooter/motor2_output",
                Shooter.getInstance().getMotor2PercentOutput());

        SmartDashboard.putNumber("climber/lift_out",
                Climber.getInstance().getLiftPercentOutput());
        SmartDashboard.putNumber("climber/right_winch_out",
                Climber.getInstance().getRightWinchPercentOutput());
        SmartDashboard.putNumber("climber/left_winch_out",
                Climber.getInstance().getLeftWinchPercentOutput());
        SmartDashboard.putBoolean("climber/bottom_limit_switch",
                Climber.getInstance().getBottomLimitSwitch());
        SmartDashboard.putBoolean("climber/top_limit_switch",
                Climber.getInstance().getTopLimitSwitch());

        SmartDashboard.putNumber("intake/ground_intake_out",
                Intake.getInstance().getGroundMotorPercentOutput());
        SmartDashboard.putNumber("intake/transfer_intake_out",
                Intake.getInstance().getUptakeMotorPercentOutput());
        SmartDashboard.putBoolean("intake/balls_retained",
                Intake.getInstance().getBallsAreRetained());

        SmartDashboard.putNumber("joysticks/throttle_value",
                OI.getInstance().getDriveJoystick().rooGetThrottle());
        SmartDashboard.putNumber("joysticks/input_x",
                OI.getInstance().getDriveJoystick().rooGetX());
        SmartDashboard.putNumber("joysticks/input_y",
                OI.getInstance().getDriveJoystick().rooGetY());
        SmartDashboard.putNumber("joysticks/input_twist",
                OI.getInstance().getDriveJoystick().rooGetTwist());
    }
}