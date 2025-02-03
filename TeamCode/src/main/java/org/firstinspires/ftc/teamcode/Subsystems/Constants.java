package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /* WRIST POSITIONS */
    public static class Wrist {
        public static final double WRIST_STOW = 0.1;
        public static final double WRIST_EXTEND_SHORT = 0.2;
        public static final double WRIST_EXTEND_MEDIUM = 0.55;
        public static final double WRIST_EXTEND_LONG = 0.8;
    }

    /* INTAKE POSITIONS */
    public static class Intake{
        public static final double INTAKE_STOW = 0.1;
        public static final double INTAKE_OPEN = 0.4;
    }

    /* ARM POSITIONS */
    public static class Arm {
        public static final double ARM_STOW = -10;

        public static final double ARM_GROUDGRAB_SHORT = -18;
        public static final double ARM_GROUDGRAB_MEDIUM = -10;
        public static final double ARM_GROUDGRAB_LONG = -5;

        public static final double ARM_CLIMB = 90;
        public static final double ARM_LOWBASKET = 45;
        public static final double ARM_AUTO_HIGHBASKET = 65;
        public static final double ARM_HIGHBASKET = 90;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 0;
        public static final double ARM_HIGHCHAMBER = 45;
    }

    /* ELEVATOR POSITIONS */
    public static class Elevator {
        public static final double ELEVATOR_STOW = 0;

        public static final double ELEVATOR_GROUDGRAB_SHORT = 30;
        public static final double ELEVATOR_GROUDGRAB_MEDIUM = 40;
        public static final double ELEVATOR_GROUDGRAB_LONG = 50;


        public static final double ELEVATOR_CLIMB = 50;
        public static final double ELEVATOR_LOWBASKET = 39;
        public static final double ELEVATOR_HIGHBASKET = 75;

        // CHAMBERS
        public static final double ELEVATOR_LOWCHAMBER = 3;
        public static final double ELEVATOR_HIGHCHAMBER = 39;

    }
}
