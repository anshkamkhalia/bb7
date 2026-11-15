package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.Poses.*;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;

public class AutoPaths {

    public static Path redPath1() {
        return Paths.curve(RED_START, RED_PATH1_CONTROL1, RED_PATH1_END)
                .linear(RED_START, RED_PATH1_END);
    }

    public static Path bluePath1() {
        return Paths.curve(BLUE_START, BLUE_PATH1_CONTROL1, BLUE_PATH1_END)
                .linear(BLUE_START, BLUE_PATH1_END);
    }

    public static Path redDownPath1() {
        return Paths.curve(RED_DOWN_START, RED_DOWN_PATH1_CONTROL1, RED_DOWN_PATH1_END)
                .linear(RED_DOWN_START, RED_DOWN_PATH1_END);
    }

    public static Path blueNextToFlowerPerpendicularToBlueHive() {
        return Paths.curve(BLUE_FLOWER_PERP_HIVE_START, BLUE_FLOWER_PERP_HIVE_CONTROL1, BLUE_FLOWER_PERP_HIVE_END)
                .linear(BLUE_FLOWER_PERP_HIVE_START, BLUE_FLOWER_PERP_HIVE_END);
    }
}