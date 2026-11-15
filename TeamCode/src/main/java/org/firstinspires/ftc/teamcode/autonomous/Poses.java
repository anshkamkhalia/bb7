package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Poses {

    private static final PoseFactory P = PoseFactory.degrees();

    // ---------- Red: In front of up cell ----------
    public static final Pose RED_START          = P.of(62.3576, 8.7409, 90);
    public static final Pose RED_PATH1_END      = P.of(0, 101.3885, 0);
    public static final Pose RED_PATH1_CONTROL1 = P.of(20.1335, 26.1145, 0);

    // ---------- Blue: In front of up cell ----------
    public static final Pose BLUE_START          = P.of(78.1916, 132.899, 270);
    public static final Pose BLUE_PATH1_END      = P.of(129.212, 45.6867, 0);
    public static final Pose BLUE_PATH1_CONTROL1 = P.of(120.6667, 106.5091, 0);

    // ---------- Red: In front of down cell ----------
    public static final Pose RED_DOWN_START          = P.of(58.3363, 134.9096, 270);
    public static final Pose RED_DOWN_PATH1_END      = P.of(9.0009, 115.9236, 0);
    public static final Pose RED_DOWN_PATH1_CONTROL1 = P.of(51.0477, 102.9905, 0);

    // BlueNextToFlowerPerpendicularToBlueHive
    public static final Pose BLUE_FLOWER_PERP_HIVE_ROBOT_START = P.of(132.9819, 77.8572, 180);
    public static final Pose BLUE_FLOWER_PERP_HIVE_START       = P.of(132.9819, 77.8572, 180);
    public static final Pose BLUE_FLOWER_PERP_HIVE_CONTROL1    = P.of(116.8967, 70.3173, 0);
    public static final Pose BLUE_FLOWER_PERP_HIVE_END         = P.of(129.6403, 45.2992, 180);
}