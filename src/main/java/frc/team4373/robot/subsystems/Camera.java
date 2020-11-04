package frc.team4373.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4373.robot.commands.camera.CameraCommand;

/**
 * A symbolic subsystem that allows for tracking and killing commands that require only the camera.
 */
public class Camera extends SubsystemBase {
    private static volatile Camera instance;

    /**
     * The getter for the Camera class.
     * @return the singleton Camera object.
     */
    public static Camera getInstance() {
        if (instance == null) {
            synchronized (Camera.class) {
                if (instance == null) {
                    instance = new Camera();
                }
            }
        }
        return instance;
    }

    @Override
    public void periodic() {

    }
}
