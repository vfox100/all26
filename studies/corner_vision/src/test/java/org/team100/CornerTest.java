package org.team100;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CornerTest {

    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * The tag coordinate system defines the corner order, which starts in the lower
     * left, and goes counter-clockwise, using the camera convention (x-right,
     * y-down).
     */
    private static final MatOfPoint2f CORNERS_FOR_HOMOGRAPHY = new MatOfPoint2f(
            new Point(-1, 1),
            new Point(1, 1),
            new Point(1, -1),
            new Point(-1, -1));

    /** Corners from the full-scale image, in case we need them. */
    @SuppressWarnings("unused")
    private static final double[] RAW_CORNERS = new double[] { //
            955, 2477, //
            1934, 1929, //
            1847, 1042, //
            649, 1465 };

    /** Homography from the full-scale image, in case we need it. */
    @SuppressWarnings("unused")
    private static final double[] RAW_HOMOGRAPHY = new double[] { //
            684, 202, 1413, //
            -60, 612, 1741, //
            0, 0, 0 };

    /**
     * Homography between the +/-1 square above and the 0.2-scaled test image.
     */
    private static final double[] EXPECTED_HOMOGRAPHY = new double[] { //
            137, 40, 282, //
            -12, 122, 349, //
            0, 0, 0 };

    /**
     * Corners in in the 0.2-scaled test image.
     */
    private static final double[] EXPECTED_CORNERS = new double[] { //
            191, 496, //
            387, 386, //
            369, 209, //
            130, 293 };

    /**
     * Verify the detection corner locations.
     */
    @Test
    void testDetectionCorners() {
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = detection.getCorners();
            assertArrayEquals(EXPECTED_CORNERS, corners, 1.0);
        }
    }

    /**
     * Verify the detection-derived homography.
     * 
     * Computing the homography is a side-effect of finding the tag.
     */
    @Test
    void testDetectionHomography() {
        try (AprilTagDetector detector = detector()) {
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] homography = detection.getHomography();
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * Verify the OpenCV computation of homography.
     */
    @Test
    void testOpenCvHomography() {
        try (AprilTagDetector detector = detector()) {
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = getOpenCvHomographyArray(corners);
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * Verify the pose using the Apriltag estimator with the detection corners and
     * homography.
     */
    @Test
    void testDetectedPose() {
        try (AprilTagDetector detector = detector()) {
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = detection.getHomography();
            // Tag size is the real tag size
            // Camera intrinsic matches GS camera and python test
            AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                    0.1651, 935, 935, 550, 310);
            AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(conf);
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.001);
        }
    }

    /**
     * Use the Apriltag pose estimator with the homography computed by OpenCV, so we
     * don't have to ship the homography from the camera.
     */
    @Test
    void testPoseWithOpenCvHomography() {
        try (AprilTagDetector detector = detector()) {
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = getOpenCvHomographyArray(corners);
            AprilTagPoseEstimator estimator = estimator();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.001);
        }
    }

    /**
     * The OpenCV "Solve PNP" methods are not terrible but not better than the
     * Apriltag pose estimator. Don't do it this way.
     */
    @Test
    void testOpenCvPose() {
        try (AprilTagDetector detector = detector()) {
            Mat img = loadVerbatim();
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = detection.getCorners();
            getOpenCvPose(corners);
        }
    }

    /**
     * Extract corners from a distorted image, undistort them, and verify that
     * they're correct. Note the slightly larger tolerance.
     */
    @Test
    void testCorrectedCorners() {
        final double k1 = -0.3;
        try (AprilTagDetector detector = detector()) {
            Mat img = distort(loadVerbatim(), k1);
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            assertArrayEquals(EXPECTED_CORNERS, corners, 2.0);
        }
    }

    /**
     * Compute the homography for the corrected corners (using OpenCV) and verify
     * that it is correct.
     */
    @Test
    void testCorrectedHomography() throws IOException {
        try (AprilTagDetector detector = detector()) {
            double k1 = -0.3;
            Mat img = distort(loadVerbatim(), k1);
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            double[] homography = getOpenCvHomographyArray(corners);
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * Verify the corrected pose.  Note the slightly larger tolerance.
     */
    @Test
    void testDetectionWithDistortion() throws IOException {
        try (AprilTagDetector detector = detector()) {
            double k1 = -0.3;
            Mat img = distort(loadVerbatim(), k1);
            AprilTagDetection detection = getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            double[] homography = getOpenCvHomographyArray(corners);
            AprilTagPoseEstimator estimator = estimator();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.003);
        }
    }

    // This is just to look at the distorted image to make sure it's working.
    // @Test
    void testDetection2() throws IOException {
        Mat distorted_img = distort(loadVerbatim(), -0.3);
        Imgcodecs.imwrite("debug.jpg", distorted_img);
    }

    //////////////////////////////////////
    //////////////////////////////////////
    //////////////////////////////////////

    /**
     * Make a detector looking for 36h11. Remember to close it.
     */
    private AprilTagDetector detector() {
        AprilTagDetector detector = new AprilTagDetector();
        detector.addFamily("tag36h11");
        return detector;
    }

    /**
     * The pose estimator uses the real tag size, and the same intrinsics as the
     * python test, which is similar to the Raspberry Pi GS camera.
     */
    private AprilTagPoseEstimator estimator() {
        AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                0.1651, 935, 935, 550, 310);
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(conf);
        return estimator;
    }

    /**
     * Return the corners from the detection, undistorted using k1.
     */
    private double[] getCorrectedCorners(final double k1, AprilTagDetection detection) {
        // the detected corners are now in different places, so we have to fix them.
        double[] detectedCorners = detection.getCorners();
        MatOfPoint2f srcCorners = new MatOfPoint2f(
                new Point(detectedCorners[0], detectedCorners[1]),
                new Point(detectedCorners[2], detectedCorners[3]),
                new Point(detectedCorners[4], detectedCorners[5]),
                new Point(detectedCorners[6], detectedCorners[7]));
        MatOfPoint2f dstCorners = new MatOfPoint2f();

        // Undistort the corner points.
        //
        // Add extra iterations to be sure? This seems not to matter for this
        // particular case but it's not a bad idea in general.
        TermCriteria term = new TermCriteria(
                TermCriteria.COUNT | TermCriteria.EPS, 40, 0.01);
        Calib3d.undistortImagePoints(
                srcCorners,
                dstCorners,
                getCameraMatrix(),
                getDistCoeffs(k1),
                term);
        Point[] dstL = dstCorners.toArray();
        double[] correctedCorners = new double[] { //
                dstL[0].x, dstL[0].y, //
                dstL[1].x, dstL[1].y, //
                dstL[2].x, dstL[2].y, //
                dstL[3].x, dstL[3].y };
        return correctedCorners;
    }

    private Mat distort(Mat undistorted_img, double k1) {
        Size size = undistorted_img.size();

        // Compute the map.
        //
        // Each point is a pointer to its undistorted location, so the
        // map really describes distortion: to get the value for a pixel,
        // use the pointer to the undistorted pixel.
        MatOfPoint2f dstPoints = new MatOfPoint2f();

        Calib3d.undistortPoints(
                getSrcPoints(size),
                dstPoints,
                getCameraMatrix(),
                getDistCoeffs(k1),
                new Mat(),
                getCameraMatrix());
        List<Point> dstList = dstPoints.toList();

        // Split the map.
        //
        // The remap function takes two maps, one for x and one for y.
        Mat mapX = new Mat(size, CvType.CV_32F);
        Mat mapY = new Mat(size, CvType.CV_32F);
        for (int row = 0; row < size.height; ++row) {
            for (int col = 0; col < size.width; ++col) {
                // row-major
                Point p = dstList.get((int) (size.width) * row + col);
                mapX.put(row, col, p.x);
                mapY.put(row, col, p.y);
            }
        }
        // Apply the map.
        //
        // The map describes the position in the src image that
        // each position in the dst image should use.
        // dst(x,y) = src(h(x,y))
        Mat dst = new Mat();
        Imgproc.remap(undistorted_img, dst, mapX, mapY, Imgproc.INTER_LINEAR);
        return dst;
    }

    /** Each src point is just its location (x, y). */
    private MatOfPoint2f getSrcPoints(Size size) {
        List<Point> points = new ArrayList<>();
        for (int row = 0; row < size.height; ++row) {
            for (int col = 0; col < size.width; ++col) {
                points.add(new Point(col, row));
            }
        }
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(points);
        return srcPoints;
    }

    private Mat getDistCoeffs(double k1) {
        Mat distCoeffs = new Mat(1, 4, CvType.CV_64FC1);
        distCoeffs.put(0, 0, k1, 0.0, 0.0, 0.0);
        return distCoeffs;
    }

    private Mat getCameraMatrix() {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, //
                935, 0, 550, //
                0, 935, 310, //
                0, 0, 1);
        return cameraMatrix;
    }

    /**
     * Find files in the classpath.
     * 
     * Test resources should go in src/test/resources,
     * and they appear in build/resources/test.
     */
    private String res(String filename) {
        // this puts "/" before "C:" on windows which does not work
        return getClass().getResource("/" + filename).getPath();
    }

    /**
     * Load the test image and scale it to 0.2 in each dimension.
     */
    private Mat loadVerbatim() {
        // TODO: fix this filename issue
        Mat img = Imgcodecs.imread(res("tag_and_board.jpg"));
        // Mat img = Imgcodecs.imread(
        // "C:/Users/joel/FRC/TRUHER/all26/studies/corner_vision/build/resources/test/tag_and_board.jpg");
        assertNotNull(img);
        Size size = img.size();
        assertEquals(5504, size.width);
        assertEquals(3096, size.height);
        // the equivalent tag_detector_test.py resizes, so we do too.
        Imgproc.resize(img, img, new Size(1100, 620));
        size = img.size();
        assertEquals(1100, size.width);
        assertEquals(620, size.height);
        return img;
    }

    private double[] getOpenCvHomographyArray(double[] corners) {
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(corners[0], corners[1]),
                new Point(corners[2], corners[3]),
                new Point(corners[4], corners[5]),
                new Point(corners[6], corners[7]));
        Mat openCvHomographyMat = Calib3d.findHomography(CORNERS_FOR_HOMOGRAPHY, dstPoints);
        double[] openCvHomographyArray = new double[9];
        openCvHomographyMat.get(0, 0, openCvHomographyArray);
        return openCvHomographyArray;
    }

    /**
     * The tag pictured in tag_and_board.jpg is tag #1, and it appears
     * right-side-up.
     * 
     * Guess reasonable pose in camera cordinates.
     * Translation coordinates are (x-right, y-down, z-forward)
     * Rotation coordinates are (x:pitch up, y:yaw right, z:roll clockwise)
     * Translation (-0.25, 0.1, 0.5)
     * Rotation (0.5, -0.4, -0.2)
     */
    private void verifyPose(Transform3d pose, double tol) {
        // sometimes inverted, sometimes not?
        Translation3d t = pose.getTranslation();
        Rotation3d r = pose.getRotation();
        assertEquals(-0.186, t.getX(), tol);
        assertEquals(0.027, t.getY(), tol);
        assertEquals(0.642, t.getZ(), tol);
        assertEquals(0.786, r.getX(), tol);
        assertEquals(-0.607, r.getY(), tol);
        // this is upposed to be a little roll to the left
        // but sometimes it pi minus that?
        assertEquals(-0.492, r.getZ(), tol);
    }

    private AprilTagDetection getDetection(AprilTagDetector detector, Mat img) {
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2GRAY);
        AprilTagDetection[] detections = detector.detect(img);
        // we find the big one
        assertEquals(1, detections.length);
        AprilTagDetection detection = detections[0];
        assertEquals(1, detection.getId());
        return detection;
    }

    /**
     * Extract the pose using openCV.
     * This sucks, the Apriltag one is better.
     */
    private void getOpenCvPose(double[] detectedCorners) {
        MatOfPoint3f obj = new MatOfPoint3f(
                new Point3(-0.08255, 0.08255, 0.0),
                new Point3(0.08255, 0.08255, 0.0),
                new Point3(0.08255, -0.08255, 0.0),
                new Point3(-0.08255, -0.08255, 0.0));
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(detectedCorners[0], detectedCorners[1]),
                new Point(detectedCorners[2], detectedCorners[3]),
                new Point(detectedCorners[4], detectedCorners[5]),
                new Point(detectedCorners[6], detectedCorners[7]));
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Calib3d.solvePnP(obj, dstPoints, getCameraMatrix(), new MatOfDouble(getDistCoeffs(0)), rvec, tvec, false,
                Calib3d.SOLVEPNP_IPPE_SQUARE);
        System.out.printf("rvec %s\n", rvec.dump());
        System.out.printf("tvec %s\n", tvec.dump());

        Transform3d openCvPose = new Transform3d(
                new Translation3d(tvec.get(0, 0)[0], tvec.get(1, 0)[0], tvec.get(2, 0)[0]),
                new Rotation3d(rvec.get(0, 0)[0], rvec.get(1, 0)[0], rvec.get(2, 0)[0]));
        System.out.printf("transform %s\n", openCvPose);
        // the translation is pretty close but the rotation is not.
        // verifyPose(openCvPose, 1, 0.001);
    }

}
