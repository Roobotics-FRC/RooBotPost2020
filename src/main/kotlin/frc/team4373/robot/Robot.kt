package frc.team4373.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.team4373.robot.commands.camera.CameraCommand
import frc.team4373.robot.commands.climber.ClimberCommand
import frc.team4373.robot.commands.drivetrain.DrivetrainCommand
import frc.team4373.robot.commands.intake.IntakeCommand
import frc.team4373.robot.commands.shooter.ShooterCommand
import frc.team4373.robot.input.OI
import frc.team4373.robot.subsystems.*


class Robot: TimedRobot() {
    override fun robotInit() {
        Camera.getInstance().defaultCommand = CameraCommand()
        Climber.getInstance().defaultCommand = ClimberCommand()
        Intake.getInstance().defaultCommand = IntakeCommand()
        Shooter.getInstance().defaultCommand = ShooterCommand()
        Drivetrain.getInstance().defaultCommand = DrivetrainCommand()
//        WheelSpinner.getInstance().defaultCommand = WheelSpinnerCommand()
    }

    override fun autonomousInit() {

        // Create config for trajectory

        // Create config for trajectory
        val config: TrajectoryConfig = TrajectoryConfig(RobotMap.MAX_SPEED_METERS_PER_SECOND,
                RobotMap.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Drivetrain.kinematics)

        // An example trajectory to follow.  All units in meters.

        // An example trajectory to follow.  All units in meters.
        val exampleTrajectory = TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
                Pose2d(0.0, 0.0, Rotation2d()),  // Pass through these two interior waypoints, making an 's' curve path
                listOf(
                        Translation2d(1.0, 1.0),
                        Translation2d(2.0, -1.0)
                ),  // End 3 meters straight ahead of where we started, facing forward
                Pose2d(3.0, 0.0, Rotation2d()),
                config
        )

        val thetaController = ProfiledPIDController(1.0, 0.0, 0.0,
                RobotMap.THETA_CONTROLLER_CONSTRAINTS)
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        val swerveControllerCommand = SwerveControllerCommand(
                exampleTrajectory,
                { Drivetrain.pose },  //Functional interface to feed supplier
                Drivetrain.kinematics,  //Position controllers
                PIDController(1.0, 0.0, 0.0),
                PIDController(1.0, 0.0, 0.0), thetaController,
                Drivetrain::setModuleStates,
                Drivetrain
        )

        swerveControllerCommand.schedule()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        SmartDashboard.putNumber("shooter/encoder_vel", Shooter.getInstance().velocity)
        SmartDashboard.putNumber("shooter/motor1_output",
                Shooter.getInstance().motor1PercentOutput)
        SmartDashboard.putNumber("shooter/motor2_output",
                Shooter.getInstance().motor2PercentOutput)

        SmartDashboard.putNumber("climber/lift_out",
                Climber.getInstance().liftPercentOutput)
        SmartDashboard.putNumber("climber/right_winch_out",
                Climber.getInstance().rightWinchPercentOutput)
        SmartDashboard.putNumber("climber/left_winch_out",
                Climber.getInstance().leftWinchPercentOutput)
        SmartDashboard.putBoolean("climber/bottom_limit_switch",
                Climber.getInstance().bottomLimitSwitch)
        SmartDashboard.putBoolean("climber/top_limit_switch",
                Climber.getInstance().topLimitSwitch)

        SmartDashboard.putNumber("intake/ground_intake_out",
                Intake.getInstance().groundMotorPercentOutput)
        SmartDashboard.putNumber("intake/transfer_intake_out",
                Intake.getInstance().uptakeMotorPercentOutput)
        SmartDashboard.putBoolean("intake/balls_retained",
                Intake.getInstance().ballsAreRetained)

        SmartDashboard.putNumber("joysticks/throttle_value",
                OI.getInstance().driveJoystick.rooGetThrottle())
        SmartDashboard.putNumber("joysticks/input_x",
                OI.getInstance().driveJoystick.rooGetX())
        SmartDashboard.putNumber("joysticks/input_y",
                OI.getInstance().driveJoystick.rooGetY())
        SmartDashboard.putNumber("joysticks/input_twist",
                OI.getInstance().driveJoystick.rooGetTwist())
    }

}