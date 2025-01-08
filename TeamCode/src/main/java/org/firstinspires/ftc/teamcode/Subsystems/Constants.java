package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /*PENDING TEST*/

    public static class Arm {
        /* ARM POSITIONS */
        public static final double ARM_STOW = 47;
        public static final double ARM_GROUNDGRAB = 44;
        public static final double ARM_CLIMB = 50; /* PENDING TEST */

        // BASKETS
        public static final double ARM_LOWBASKET = 95;
        public static final double ARM_HIGHBASKET = 95;

        // CHAMBERS
        public static final double ARM_LOWCHAMBER = 0;
        public static final double ARM_HIGHCHAMBER = 15;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    public static class Elevator {
        /* ELEVATOR POSITIONS */
        public static final double ELEVATOR_STOW = 0;
        public static final double ELEVATOR_GROUNDGRAB = 10;
        public static final double ELEVATOR_CLIMB = 50; /* PENDING TEST */

        // BASKETS
        public static final double ELEVATOR_LOWBASKET = 6;
        public static final double ELEVATOR_HIGHBASKET = 55;

        // CHAMBERS
        public static final double ELEVATOR_LOWCHAMBER = 0;
        public static final double ELEVATOR_HIGHCHAMBER = 5;//0 si se hace de arriba-abajo y 5 si se hace de abajo-arriba

    }
}
