// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.TimeSyncEventData;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    IntegerPublisher servernowpub;
    NetworkTableListener listener;

    public Robot() {
        NetworkTableInstance.getDefault().startServer();
        servernowpub = NetworkTableInstance.getDefault()
                .getIntegerTopic("servernow").publish();
        listener = NetworkTableListener.createTimeSyncListener(
                NetworkTableInstance.getDefault(), true, this::consume);
    }

    @Override
    public void robotPeriodic() {
        servernowpub.set(RobotController.getFPGATime());
        NetworkTableInstance.getDefault().flush();
        if (!listener.waitForQueue(10)) {
            System.out.println("wait timed out");
        }
    }

    void consume(NetworkTableEvent e) {
        if (e.is(Kind.kTimeSync)) {
            TimeSyncEventData tsed = e.timeSyncData;
            System.out.printf("Time Sync Event: %d %d %b\n",
                    tsed.serverTimeOffset, tsed.rtt2, tsed.valid);
        } else {
            System.out.printf("Wrong event type");
        }
    }

}
