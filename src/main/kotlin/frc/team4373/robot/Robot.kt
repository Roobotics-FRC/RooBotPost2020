package frc.team4373.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4373.robot.commands.auton.RamThenShootAuton
import frc.team4373.robot.commands.util.ClearSubsystemCommand
import frc.team4373.robot.input.OI
import frc.team4373.robot.subsystems.Climber
import frc.team4373.robot.subsystems.Drivetrain
import frc.team4373.robot.subsystems.Intake
import frc.team4373.robot.subsystems.Shooter
import frc.team4373.swerve.SwerveDrivetrain

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot: TimedRobot() {
    var autonCommand: Command? = null

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     *
     *
     * All SmartDashboard fields should be initially added here.
     */
    override fun robotInit() {
        Climber.getInstance()
        Intake.getInstance()
        Shooter.getInstance()
        // WheelSpinner.getInstance();
        Drivetrain.getInstance()
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want run during disabled,
     * autonomous, teleoperated, and test.
     *
     *
     * This runs after the mode-specific periodic functions but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        SmartDashboard.putNumber("shooter/encoder_vel", Shooter.velocity)
        SmartDashboard.putNumber(
            "shooter/motor1_output",
            Shooter.motor1PercentOutput
        )
        SmartDashboard.putNumber(
            "shooter/motor2_output",
            Shooter.motor2PercentOutput
        )
        SmartDashboard.putNumber(
            "climber/lift_out",
            Climber.getInstance().liftPercentOutput
        )
        SmartDashboard.putNumber(
            "climber/right_winch_out",
            Climber.getInstance().rightWinchPercentOutput
        )
        SmartDashboard.putNumber(
            "climber/left_winch_out",
            Climber.getInstance().leftWinchPercentOutput
        )
        SmartDashboard.putBoolean(
            "climber/bottom_limit_switch",
            Climber.getBottomLimitSwitch()
        )
        SmartDashboard.putBoolean(
            "climber/top_limit_switch",
            Climber.getTopLimitSwitch()
        )
        SmartDashboard.putNumber(
            "intake/ground_intake_out",
            Intake.getInstance().groundMotorPercentOutput
        )
        SmartDashboard.putNumber(
            "intake/transfer_intake_out",
            Intake.getInstance().uptakeMotorPercentOutput
        )
        SmartDashboard.putBoolean(
            "intake/balls_retained",
            Intake.getInstance().ballsAreRetained
        )
        SmartDashboard.putNumber(
            "joysticks/throttle_value",
            OI.getInstance().driveJoystick.rooGetThrottle()
        )
        SmartDashboard.putNumber(
            "joysticks/input_x",
            OI.getInstance().driveJoystick.rooGetX()
        )
        SmartDashboard.putNumber(
            "joysticks/input_y",
            OI.getInstance().driveJoystick.rooGetY()
        )
        SmartDashboard.putNumber(
            "joysticks/input_twist",
            OI.getInstance().driveJoystick.rooGetTwist()
        )


        // SmartDashboard.putString("spinner/color",
        //         WheelSpinner.getInstance().getColor().toString());
        // SmartDashboard.putNumber("spinner/spinner_out",
        //         WheelSpinner.getInstance().getWheelSpinnerPercentOutput());
    }

    /**
     * This function is called once when Sandstorm mode starts.
     */
    override fun autonomousInit() {
        Drivetrain.getInstance().brakeMode = SwerveDrivetrain.BrakeMode.NONE
        autonCommand = RamThenShootAuton()
        (autonCommand as Command).start()
    }

    /**
     * This function is called once when teleoperated mode starts.
     */
    override fun teleopInit() {
        Drivetrain.getInstance().brakeMode = SwerveDrivetrain.BrakeMode.IMPLODE
        // Ensure subsystems are in a known, safe state
        // new ResetSpinnerStateCommand().start();
        ClearSubsystemCommand(Intake.getInstance()).start()
    }

    /**
     * This function is called periodically during Sandstorm mode.
     */
    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {}

    /**
     * This function runs periodically in disabled mode.
     * It is used to verify that the selected auton configuration is legal.
     * <br></br>
     * **UNDER NO CIRCUMSTANCES SHOULD SUBSYSTEMS BE ACCESSED OR ENGAGED IN THIS METHOD.**
     */
    override fun disabledPeriodic() {}
}