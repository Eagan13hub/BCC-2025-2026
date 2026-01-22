package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class X25Config {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(1024,16,0,0);
}
