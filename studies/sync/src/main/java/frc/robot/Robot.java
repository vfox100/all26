// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.TimeSyncEventData;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    IntegerPublisher servernowpub;
    NetworkTableListenerPoller poller;

    public Robot() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();
        servernowpub = inst.getIntegerTopic("servernow").publish();
        poller = new NetworkTableListenerPoller(inst);
        poller.addTimeSyncListener(false);
    }

    @Override
    public void robotPeriodic() {
        servernowpub.set(RobotController.getFPGATime());
        for (NetworkTableEvent e : poller.readQueue()) {
            TimeSyncEventData tsed = e.timeSyncData;
            System.out.printf("Time Sync Event: %d %d %b\n",
                    tsed.serverTimeOffset, tsed.rtt2, tsed.valid);
        }
        NetworkTableInstance.getDefault().flush();
    }
}
