package org.team100;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
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

    double[] RAW_CORNERS = new double[] { //
            955, 2477, //
            1934, 1929, //
            1847, 1042, //
            649, 1465 };
    double[] RAW_HOMOGRAPHY = new double[] { //
            684, 202, 1413, //
            -60, 612, 1741, //
            0, 0, 0 };

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

    @Test
    void testDetection() throws IOException {
        OpenCvLoader.forceLoad();
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            // TODO: fix this filename issue
            // Mat img = Imgcodecs.imread(res("tag_and_board.jpg"));
            Mat img = Imgcodecs.imread(
                    "C:/Users/joel/FRC/TRUHER/all26/studies/corner_vision/build/resources/test/tag_and_board.jpg");
            assertNotNull(img);
            Size size = img.size();
            assertEquals(5504, size.width);
            assertEquals(3096, size.height);
            // the equivalent tag_detector_test.py resizes, so we do too.
            Imgproc.resize(img, img, new Size(1100, 620));
            size = img.size();
            assertEquals(1100, size.width);
            assertEquals(620, size.height);
            Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2GRAY);
            AprilTagDetection[] detections = detector.detect(img);
            // we find the big one
            assertEquals(1, detections.length);
            AprilTagDetection detection = detections[0];
            assertEquals(1, detection.getId());

            // Verify the corner locations.
            double[] corners = new double[] { //
                    191, 496, //
                    387, 386, //
                    369, 209, //
                    130, 293 };
            assertArrayEquals(corners, detection.getCorners(), 1.0);

            // Verify the computed homography for those corners.
            double[] homography = new double[] { //
                    137, 40, 282, //
                    -12, 122, 349, //
                    0, 0, 0 };
            assertArrayEquals(homography, detection.getHomography(), 1.0);

            // Compute the homography again with OpenCV: it's the same.
            MatOfPoint2f srcPoints = new MatOfPoint2f(
                    new Point(-1, 1),
                    new Point(1, 1),
                    new Point(1, -1),
                    new Point(-1, -1));
            MatOfPoint2f dstPoints = new MatOfPoint2f(
                    new Point(corners[0], corners[1]),
                    new Point(corners[2], corners[3]),
                    new Point(corners[4], corners[5]),
                    new Point(corners[6], corners[7]));
            Mat h2 = Calib3d.findHomography(srcPoints, dstPoints);
            double[] d2 = new double[9];
            h2.get(0, 0, d2);
            assertArrayEquals(homography, d2, 1.0);

            // tag size is the real tag size
            // camera intrinsic matches GS camera and python test
            AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                    0.1651, 935, 935, 550, 310);
            AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(conf);
            Transform3d pose = estimator.estimate(homography, corners);
            Translation3d t = pose.getTranslation();
            Rotation3d r = pose.getRotation();
            // these are all inverted compared to the python code.
            assertEquals(0.186, t.getX(), 0.001);
            assertEquals(-0.027, t.getY(), 0.001);
            assertEquals(-0.642, t.getZ(), 0.001);
            assertEquals(-0.786, r.getX(), 0.001);
            assertEquals(0.600, r.getY(), 0.001); // different in last digit
            assertEquals(Math.PI - 0.492, r.getZ(), 0.003); // different in last digit

        }
    }

    @Test
    void testDetection2() throws IOException {
        OpenCvLoader.forceLoad();
        Mat img = Imgcodecs.imread(
                "C:/Users/joel/FRC/TRUHER/all26/studies/corner_vision/build/resources/test/tag_and_board.jpg");
        redistort(img);
    }

    Mat redistort(Mat undistorted_img) {
        List<Point> points = new ArrayList<>();
        for (int row = 0; row < undistorted_img.height(); ++row) {
            for (int col = 0; col < undistorted_img.width(); ++col) {
                points.add(new Point(col, row));
            }
        }
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(points);

        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, //
                935, 0, 550, //
                0, 935, 310, //
                0, 0, 1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        distCoeffs.put(0, 0, -0.1, 0.0, 0.0, 0.0);

        MatOfPoint2f dstPoints = new MatOfPoint2f();
        Calib3d.undistortPoints(
                srcPoints,
                dstPoints,
                cameraMatrix,
                distCoeffs,
                new Mat(),
                cameraMatrix);
        System.out.println(dstPoints);
        // Mat mapX = new Mat(undistorted_img.size(), CvType.CV_32F);
        // Mat mapY = new Mat(undistorted_img.size(), CvType.CV_32F);

        Mat dst = new Mat();
        // Imgproc.remap(undistorted_img, dst, mapX, mapY, Imgproc.INTER_LINEAR);
        return dst;
    }

    /**
     * One option is to ship *raw* camera corners, and for the camera to be unaware
     * of its own parameters.
     */
    // @Test
    void testUndistortInJava() throws IOException {
        OpenCvLoader.forceLoad();
        // from the camera
        Mat src = new Mat();
        Mat dst = new Mat();
        Mat intrinsic = new Mat();
        Mat distortion = new Mat();
        TermCriteria term = new TermCriteria(
                TermCriteria.COUNT | TermCriteria.EPS, 40, 0.01);
        Calib3d.undistortImagePoints(src, dst, intrinsic, distortion, term);
        AprilTagPoseEstimator.Config c = new AprilTagPoseEstimator.Config(
                0.1651, 100, 100, 50, 50);
        AprilTagPoseEstimator e = new AprilTagPoseEstimator(c);
        // do we have to ship this?
        double[] homography = new double[9];
        double[] corners = new double[8];
        Transform3d t = e.estimate(homography, corners);
        assertEquals(new Transform3d(), t);
    }

    /** Instead of the tag-detector solver, we can use the OpenCV one. */
    // @Test
    void testSolvePNP() throws IOException {
        OpenCvLoader.forceLoad();
        MatOfPoint3f obj = new MatOfPoint3f();
        MatOfPoint2f img = new MatOfPoint2f();
        Mat intrinsic = new Mat();
        MatOfDouble distortion = new MatOfDouble();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Calib3d.solvePnP(obj, img, intrinsic, distortion, rvec, tvec, false, Calib3d.SOLVEPNP_IPPE_SQUARE);
        assertEquals(new Mat(), rvec);
        assertEquals(new Mat(), tvec);
    }
}
