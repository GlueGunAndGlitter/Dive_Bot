package frc.robot;

public final class ApriltagConstants {

    public static final class Tag {
        public final double x;
        public final double y;

        public Tag(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }


    public static final class redReef {
    
        public static final Tag TAG_6 = new Tag(13.474446, 3.306318);
        public static final Tag TAG_7 = new Tag(13.890498, 4.0259);
        public static final Tag TAG_8 = new Tag(13.474446, 4.745482);
        public static final Tag TAG_9 = new Tag(12.643358, 4.745482);
        public static final Tag TAG_10 = new Tag(12.227306, 4.0259);
        public static final Tag TAG_11 = new Tag(12.643358, 3.306318);


    }
    public static final class blueReef {
        
        public static final Tag TAG_17 = new Tag(4.073906, 3.306318);
        public static final Tag TAG_18 = new Tag(3.6576, 4.0259);
        public static final Tag TAG_19 = new Tag(4.073906, 4.745482);
        public static final Tag TAG_20 = new Tag(4.90474, 4.745482);
        public static final Tag TAG_21 = new Tag(5.321046, 4.0259);
        public static final Tag TAG_22 = new Tag(4.90474, 3.306318);
        
    }

    public static final class redfeeder{
    
        
        public static final Tag TAG_1 = new Tag(16.697198, 0.65532);
        public static final Tag TAG_2 = new Tag(16.697198, 7.39648);
        
    }

    public static final class bluefeeder{
        
        public static final Tag TAG_12 = new Tag(0.851154, 0.65532);
        public static final Tag TAG_13 = new Tag(0.851154, 7.39648);
        
    }

    public static final Tag TAG_3 = new Tag(11.56081, 8.05561);
    public static final Tag TAG_4 = new Tag(9.27608, 6.137656);
    public static final Tag TAG_5 = new Tag(9.27608, 1.914906);
    public static final Tag TAG_14 = new Tag(8.272272, 6.137656);
    public static final Tag TAG_15 = new Tag(8.272272, 1.914906);
    public static final Tag TAG_16 = new Tag(5.987542, -0.00381);

    // Private constructor to prevent instantiation
    private ApriltagConstants() {
    }
}
