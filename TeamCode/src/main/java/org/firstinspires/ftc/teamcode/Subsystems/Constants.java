package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /*PENDING TEST*/

    public static class Arm {
        /* ARM POSITIONS */
        public static final double ARM_STOW = -20;
        public static final double ARM_GROUNDGRAB = -15;
        public static final double ARM_CLIMB = 90; /* PENDING TEST */

        // BASKETS
        public static final double ARM_LOWBASKET = 88;
        public static final double ARM_HIGHBASKET = 88;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 89;
        public static final double ARM_HIGHCHAMBER = 89;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    public static class Elevator {
        /* ELEVATOR POSITIONS */
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
