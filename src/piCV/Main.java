package piCV;

import java.io.IOException;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

public class Main {
	public static void main(String[] args) throws IOException, InterruptedException{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Mat mat = Mat.eye(3, 3,CvType.CV_8UC1);
		System.out.println("mat = " + mat.dump());
		/*File input = new File("grabcut.jpg");
		BufferedImage image = ImageIO.read(input);
		*/
		for (int i = 0;i<1;i++){

			VideoCapture cap = new VideoCapture(i);
			Mat frame = new Mat();
			if (cap.isOpened()==false){
				System.out.println(i+": Not open");
				frame=Imgcodecs.imread("4.jpg");
			} else {
				System.out.println(i+": Valid");
				System.out.println(i+": Webcam: "+cap.toString());
				cap.read(frame);
			}
			Imgcodecs.imwrite(i+"test.jpg", frame);
		}
	}
}
