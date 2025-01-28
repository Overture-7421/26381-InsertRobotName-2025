package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /* WRIST POSITIONS */
    public static class Wrist {
        public static final double WRIST_STOW = 0;
        public static final double WRIST_EXTEND = 0.6;
        public static final double WRIST_GROUDGRAB = 0.6;
        public static final double WRIST_HIGHBASKET = 0.3;
        public static final double WRIST_LOWBASKET = 0.3;
        public static final double WRIST_LOWCHAMBER = 0.3; //PENDING
        public static final double WRIST_HIGHCHAMBER = 0.3; //PENDING
    }

    /* ARM POSITIONS */
    public static class Arm {
        public static final double ARM_STOW = -20;
        public static final double ARM_GROUNDGRAB = -17;
        public static final double ARM_CLIMB = 90; /* PENDING TEST */

        // BASKETS
        public static final double ARM_LOWBASKET = 88;
        public static final double ARM_HIGHBASKET = 88;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 89;
        public static final double ARM_HIGHCHAMBER = 89;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    /* ELEVATOR POSITIONS */
    public static class Elevator {
        public static final double ELEVATOR_STOW = 0;
        public static final double ELEVATOR_GROUNDGRAB = 30;
        public static final double ELEVATOR_CLIMB = 50; /* PENDING TEST */

        // BASKETS
        public static final double ELEVATOR_LOWBASKET = 39;
        public static final double ELEVATOR_HIGHBASKET = 69;

        // CHAMBERS
        public static final double ELEVATOR_LOWCHAMBER = 3;
        public static final double ELEVATOR_HIGHCHAMBER = 36;//0 si se hace de arriba-abajo y 5 si se hace de abajo-arriba

    }
}
