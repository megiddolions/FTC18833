package org.firstinspires.ftc.teamcode.vison.pipelines.align;

public enum VisionTarget {
    RedTower,
    BlueTower,
    RedWobell,
    BlueWobell,
    BluePowerShoots,
    RedPowerShoots,
    RingStack,
    None;

    public enum PowerShoot {
        Left(0),
        Center(1),
        Right(2);

        int offset;
        PowerShoot(int offset) {
            this.offset = offset;
        }

        public PowerShoot left() {
            switch (this) {
                case Left:
                    return Right;
                case Right:
                    return Center;
                case Center:
                    return Left;
            }
            return Left;
        }

        public PowerShoot right() {
            switch (this) {
                case Right:
                    return Left;
                case Left:
                    return Center;
                case Center:
                    return Right;
            }
            return Left;
        }
    }
}
