import java.sql.Blob;
import java.util.ArrayList;
import java.util.Iterator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.MatOfDouble;
import org.opencv.calib3d.Calib3d;
public class PiTrack {
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	public static final Scalar 
	RED = new Scalar(0, 0, 255),
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	BLACK = new Scalar(0,0,0),
	YELLOW = new Scalar(0, 255, 255),
	LOWER_BOUNDS = new Scalar(60,40,150),
	UPPER_BOUNDS = new Scalar(120,255,255);
	
	public static final int CAMERA_PORT=0;
	
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, matHeirarchy;
	public static final int TARGET_WIDTH=20;
	public static final int TARGET_HEIGHT=14;
	
	public static boolean shouldRun = true;

	public static Mat matDefault = Mat.eye(3, 3, 4);
	public static Mat intrinsicParamaters = Mat.eye(3, 3, 4);
	public static MatOfDouble distortionCoefficients = new MatOfDouble(0.05194245, -0.57580072,  0.00179718, -0.00758639,  0.71055324);
	
	public static NetworkTable table;
	public static void main(String[] args) {
		NetworkTable.setServerMode();
		table = NetworkTable.getTable("imgdata");
		
		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		matHeirarchy = new Mat();
		VideoCapture videocapture = new VideoCapture(0);
		videocapture.open(CAMERA_PORT);
		
		matDefault.put(0, 0, 999);
		matDefault.put(1, 0, 999);
		matDefault.put(2, 0, 999);
		
		intrinsicParamaters.put(0, 0, 646.27819274);
		intrinsicParamaters.put(0, 1, 0);
		intrinsicParamaters.put(0, 2, 310.42911043);
		intrinsicParamaters.put(1, 0, 0);
		intrinsicParamaters.put(1, 1, 648.07372911);
		intrinsicParamaters.put(1, 2, 230.8119322);
		intrinsicParamaters.put(2, 0, 0);
		intrinsicParamaters.put(2, 1, 0);
		intrinsicParamaters.put(2, 2, 1);
		System.out.println("Now running PiTrack");
		while (shouldRun){
			while(videocapture.isOpened()==false){
				sendData(matDefault);
			}
			videocapture.read(matOriginal);
			sendData(processImage());			
		}
	}
	public static Mat processImage(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		
		Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);		
		Imgcodecs.imwrite("hsv.jpg", matHSV);
		Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
		Imgcodecs.imwrite("thresh.jpg", matThresh);
		Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
			MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
			Rect rec = Imgproc.boundingRect(matOfPoint);
				if(rec.height < 25 || rec.width < 25){
					iterator.remove();
				continue;
				}
				float aspect = (float)rec.width/(float)rec.height;
				if(aspect < 0.8)
					iterator.remove();
			}
			for(MatOfPoint mop : contours){
				Rect rec = Imgproc.boundingRect(mop);
				Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
		}
			double area1 = 0;
			double area2 = 0;
			Rect rec1,rec2;
		while(contours.size()>1){
			rec1=Imgproc.boundingRect(contours.get(0));
			rec2=Imgproc.boundingRect(contours.get(1));
			area1 = rec1.area();
			area2 = rec2.area();
			if (area1>area2){
				contours.remove(1);
			} else if(area2>=area1){
				contours.remove(0);
			}
		}
		if (contours.size()==0){
			return matDefault;
		}
		Rect rec = Imgproc.boundingRect(contours.get(0));
		
		Point[] target = contours.get(0).toArray();
		
		Point rectr=new Point(rec.br().x,rec.tl().y);
		Point recbl=new Point(rec.tl().x,rec.br().y);
		
		Point tl = new Point(9999,9999);
		Point tr = new Point(9999,9999);
		Point bl = new Point(9999,9999);
		Point br = new Point(9999,9999);
		for (int i=0;i<target.length;i++){
			if (calculateDistance(rec.tl(),target[i])<=calculateDistance(rec.tl(),tl)){
				tl = target[i];
			} else if (calculateDistance(rectr,target[i])<=calculateDistance(rectr,tr)){
				tr = target[i];
			} else if (calculateDistance(recbl,target[i])<=calculateDistance(recbl,bl)){
				bl = target[i];
			} else if (calculateDistance(rec.br(),target[i])<=calculateDistance(rec.br(),br)){
				br = target[i];
			}
		}
				
		MatOfPoint2f recCorners2D = new MatOfPoint2f(tl,tr,br,bl);
		MatOfPoint3f recCorners3D = new MatOfPoint3f(new Point3(-TARGET_WIDTH/2, -TARGET_HEIGHT/2, 0.0),new Point3(TARGET_WIDTH/2, -TARGET_HEIGHT/2, 0.0),new Point3(TARGET_WIDTH/2, TARGET_HEIGHT/2, 0.0),new Point3(-TARGET_WIDTH/2, TARGET_HEIGHT/2, 0.0));
		Mat rotation = new Mat();
		Mat position = new Mat();
		Calib3d.solvePnP(recCorners3D, recCorners2D, intrinsicParamaters, distortionCoefficients, rotation, position);
		
		return position;
	}
	public static void sendData(Mat position){
		/*
		table.putNumber("AngleH", position.get(0, 0)[0]);
		table.putNumber("AngleV", position.get(1,0)[0]);
		table.putNumber("Distance", position.get(2,0)[0]);
		*/
		System.out.println("AngleH: "+position.get(0, 0)[0]);
		System.out.println("AngleV: "+position.get(1, 0)[0]);
		System.out.println("Distance: "+position.get(2, 0)[0]);
		
	}
	public static double calculateDistance(Point a, Point b){
		double xdist = Math.abs(a.x-b.x);
		double ydist = Math.abs(a.y-b.y);
		return Math.sqrt((xdist*xdist)+(ydist*ydist));
	}
}