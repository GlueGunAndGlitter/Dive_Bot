// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
/** Add your docs here. */
public class ObjectDetection {
    public final PhotonCamera objectCamera;
    


    public ObjectDetection(){
        objectCamera = new PhotonCamera("AlgeCam");

    }

    public boolean isTarget(){
        var result = objectCamera.getLatestResult();
        return result.hasTargets();
    }

    public double getYaw(){
        var result = objectCamera.getLatestResult();
        if (isTarget()) {
            return result.getBestTarget().yaw;
        } else {
            return 0;
        }
    }
}

