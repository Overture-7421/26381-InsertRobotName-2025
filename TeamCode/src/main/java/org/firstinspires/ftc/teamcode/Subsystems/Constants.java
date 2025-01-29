package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /* WRIST POSITIONS */
    public static class Wrist {
        public static final double WRIST_STOW = 0;
        public static final double WRIST_EXTEND_SHORT = 1.0;
        public static final double WRIST_EXTEND_MEDIUM = 1.5;
        public static final double WRIST_EXTEND_LONG = 2.0;
    }

    /* ARM POSITIONS */
    public static class Arm {
        public static final double ARM_STOW = -20;

        public static final double ARM_GROUDGRAB_SHORT = -10;
        public static final double ARM_GROUDGRAB_MEDIUM = -15;
        public static final double ARM_GROUDGRAB_LONG = -20;

        public static final double ARM_CLIMB = 90;
        public static final double ARM_LOWBASKET = 88;
        public static final double ARM_HIGHBASKET = 88;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 89;
        public static final double ARM_HIGHCHAMBER = 89;
    }

    /* ELEVATOR POSITIONS */
    public static class Elevator {
        public static final double ELEVATOR_STOW = 0;

        public static final double ELEVATOR_GROUDGRAB_SHORT = 13;
        public static final double ELEVATOR_GROUDGRAB_MEDIUM = 18;
        public static final double ELEVATOR_GROUDGRAB_LONG = 28;


        public static final double ELEVATOR_CLIMB = 50;
        public static final double ELEVATOR_LOWBASKET = 39;
        public static final double ELEVATOR_HIGHBASKET = 69;

        // CHAMBERS
        public static final double ELEVATOR_LOWCHAMBER = 3;
        public static final double ELEVATOR_HIGHCHAMBER = 36;

    }
}
