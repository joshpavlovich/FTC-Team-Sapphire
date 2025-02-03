package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

private const val DRIVE_SPEED: Double = 0.4

private const val SECONDS_TO_OBSERVATION_ZONE_FROM_START: Double = 1.0

@Autonomous(name = "Team Sapphire: SHORT Autonomous", group = "Robot")
class ShortParkingMecanumAutonomous : ParkingMecanumAutonomous() {

    override fun getDriveSpeed(): Double {
        return DRIVE_SPEED
    }

    override fun getSecondsToObservationZoneFromStart(): Double {
        return SECONDS_TO_OBSERVATION_ZONE_FROM_START
    }
}