import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

public class AutoAlgo1 {
	
	int map_size = 3000;
	enum PixelState {blocked,explored,unexplored,visited}
	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;
	
	ArrayList<Point> points;
	long start =0;
	
	
	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;
	
	boolean isSpeedUp = false;
	
	Graph mGraph = new Graph();
	
	CPU ai_cpu;
	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func =  new ArrayList<>();
		points = new ArrayList<Point>();
		
		drone = new Drone(realMap);
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);

		
		initMap();
		
		isRotating = 0;
		ai_cpu = new CPU(200,"Auto_AI");
		ai_cpu.addFunction(this::update);
	}
	
	public void initMap() {

		map = new PixelState[map_size][map_size];
		for(int i=0;i<map_size;i++) {
			for(int j=0;j<map_size;j++) {
				map[i][j] = PixelState.unexplored;
			}
		}
		
		droneStartingPoint = new Point(map_size/2,map_size/2);
	}
	
	public void play() {
		drone.play();
		ai_cpu.play();
	}

	
	public void update(int deltaTime) {
//		System.out.println("aaaaaaaaaaaaaaaa   "+deltaTime);
		updateVisited();
		updateMapByLidars();

		ai(deltaTime);
		System.out.println(getTime());
		if(start!=0&&getTime()==4){
			System.out.println(getTime());
			SimulationWindow.return_home=true;
			SimulationWindow.backHome=false;
			SimulationWindow.lowBattery=true;
			removeNonRelevant();
			speedDown();
			spinBy((360-(int)drone.getGyroRotation())+drone.getOpticalSensorLocation().getAngle(getLastPoint()), true, new Func() {
				@Override
				public void method() {
					try {
						Thread.sleep(1000);
					} catch (InterruptedException interruptedException) {
						interruptedException.printStackTrace();
					}
					speedUp();
				}
			});
		}else if (start!=0&&getTime()>=8) {
			SimulationWindow.lowBattery=true;
			System.out.println("Low battery");
			SimulationWindow.toogleStop=true;
			System.exit(0);
		}
		
		if(isRotating != 0) {
			updateRotating(deltaTime);
		}
		if(isSpeedUp) {
			drone.speedUp(deltaTime*5);
		} else {
			drone.slowDown(deltaTime*5);
		}
		
	}
	
	public void speedUp() {
		isSpeedUp = true;
	}
	
	public void speedDown() {
		isSpeedUp = false;
	}
	
	public void updateMapByLidars() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		for(int i=0;i<drone.lidars.size();i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			//rotation = Drone.formatRotation(rotation);
			for(int distanceInCM=0;distanceInCM < lidar.current_distance;distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x,p.y,PixelState.explored);
			}
			
			if(lidar.current_distance > 0 && lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x,p.y,PixelState.blocked);
				//fineEdges((int)p.x,(int)p.y);
			}
		}
	}
	
	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		setPixel(fromPoint.x,fromPoint.y,PixelState.visited);
			
	}
	
	public void setPixel(double x, double y,PixelState state) {
		int xi = (int)x;
		int yi = (int)y;
		
		if(state == PixelState.visited) {
			map[xi][yi] = state; 
			return;
		}
		
		if(map[xi][yi] == PixelState.unexplored) {
			map[xi][yi] = state; 
		}
	}

	
	public void paintBlindMap(Graphics g) {
		Color c = g.getColor();
		
		int i = (int)droneStartingPoint.y - (int)drone.startPoint.x;
		int startY = i;
		for(;i<map_size;i++) {
			int j = (int)droneStartingPoint.x - (int)drone.startPoint.y;
			int startX = j;
			for(;j<map_size;j++) {
				if(map[i][j] != PixelState.unexplored)  {
					if(map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					} 
					else if(map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					}
					else if(map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i-startY, j-startX, i-startY, j-startX);
				}
			}
		}
		g.setColor(c);
	}

	public void paintPoints(Graphics g) {
		for(int i=0;i<points.size();i++) {
			Point p = points.get(i);
			g.drawOval((int)p.x + (int)drone.startPoint.x - 10, (int)p.y + (int)drone.startPoint.y-10, 20, 20);
		}
		
	}
	
	public void paint(Graphics g) {
		if(SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}
		
		paintBlindMap(g);
		paintPoints(g);
		
		drone.paint(g);
		
		
	}
	
	boolean is_init = true;
	double lastFrontLidarDis = 0;
	boolean isRotateRight = false;
	double changedRight = 0;
	double changedLeft = 0;
	boolean tryToEscape = false;
	int leftOrRight = 1;
	

	double max_rotation_to_direction = 20;
	boolean  is_finish = true;
	boolean isLeftRightRotationEnable = true;
	
	
	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double  risky_dis = 0;
	int max_angle_risky = 10;
	
	boolean is_lidars_max = false;
	
	double save_point_after_seconds = 3;
	
	double max_distance_between_points = 100;
	
	boolean start_return_home = false;
	
	Point init_point;
	public boolean goHomeFirst=false;
	public void removeNonRelevant(){
		var curr = drone.getOpticalSensorLocation();
		if(points.size()>=3) {
			int t=0;
			for ( int i = points.size() - 2; i > 0; i--) {
				var dis_prev=Tools.getDistanceBetweenPoints(points.get(i+1),curr);
				var dis_curr=Tools.getDistanceBetweenPoints(points.get(i),curr);
				var dis_next=Tools.getDistanceBetweenPoints(points.get(i-1),curr);
				if(dis_curr<dis_next && dis_curr<dis_prev){
					t=i;
					break;
				}
			}
			if (t!=0){
				int m=points.size();
				if (m-t>=8) {
					System.out.println(t);
					for (int i = t + 1; i < m; i++) {
						removeLastPoint();
					}
				}
			}
		}
	}

	public void ai(int deltaTime) {
//		System.out.println("_____________");
		if(!SimulationWindow.toogleAI) {
			System.out.println("12345678");
			return;
		}
	
		
		if(is_init) {
			System.out.println("amichai");
			speedUp();
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			points.add(dronePoint);
			mGraph.addVertex(dronePoint);
			is_init = false;
			start= System.currentTimeMillis();
		}
		Point dronePoint = drone.getOpticalSensorLocation();
		if(goHomeFirst){
			speedDown();
			spinBy((360-(int)drone.getGyroRotation())+dronePoint.getAngle(getLastPoint()));
			System.out.println((360-(int)drone.getGyroRotation())+dronePoint.getAngle(getLastPoint()));
			System.out.println((int)drone.getGyroRotation());
			speedUp();
			goHomeFirst=false;
		}
		if(isLeftRightRotationEnable) {
//			doLeftRight();
		}


		if(SimulationWindow.return_home) {
//			System.out.println("liav");
//			spinBy(dronePoint.getAngle(getLastPoint()));
			if( Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) <  35) {
				if(points.size() <= 1 && Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) <  35) {
					speedDown();
					System.out.println("omer");
					SimulationWindow.return_home=false;
					SimulationWindow.toogleAI=false;
					SimulationWindow.backHome=true;
					CPU.stopAllCPUS();

					SimulationWindow.updteHome();

				} else {
//
//
					removeLastPoint();
////					speedUp();
				}
			}
		} else {
			if( Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >=  max_distance_between_points) {
				points.add(dronePoint);
				mGraph.addVertex(dronePoint);
			}
		}


		Lidar forward = drone.lidars.get(0);
		if (forward.current_distance <= max_risky_distance) {
			is_risky = true;
			risky_dis = forward.current_distance;

		}


		Lidar right = drone.lidars.get(1);
		if (right.current_distance <= max_risky_distance / 3) {
			is_risky = true;
		}

		Lidar left = drone.lidars.get(2);
		if (left.current_distance <= max_risky_distance / 3) {
			is_risky = true;
		}


			if (!is_risky) {

				if (SimulationWindow.return_home) {
					if (!points.isEmpty()) {
						double rotation = Tools.getRotationBetweenPoints(dronePoint, points.get(points.size() - 1));
						spinBy(rotation);
					}
				} else if (forward.current_distance > right.current_distance && forward.current_distance > left.current_distance)
					if (drone.getSpeed() != WorldParams.max_speed) speedUp();

			} else {
				if (!try_to_escape) {
					try_to_escape = true;

					double a = right.current_distance;

					double b = left.current_distance;


					int spin_by = max_angle_risky;


					if (a > 270 && b > 270) {
						is_lidars_max = true;
						Point l1 = Tools.getPointByDistance(dronePoint, right.degrees + drone.getGyroRotation(), right.current_distance);
						Point l2 = Tools.getPointByDistance(dronePoint, left.degrees + drone.getGyroRotation(), left.current_distance);
						Point last_point = getAvgLastPoint();
						double dis_to_lidar_right = Tools.getDistanceBetweenPoints(last_point, l1);
						double dis_to_lidar_left = Tools.getDistanceBetweenPoints(last_point, l2);

						if (SimulationWindow.return_home) {
							if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < 35) {
								removeLastPoint();
							}
						} else {
							if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
								points.add(dronePoint);
								mGraph.addVertex(dronePoint);
							}
						}

						spin_by = 90;
						if (SimulationWindow.return_home) {

							spin_by *= -1;
						}


						if (dis_to_lidar_right < dis_to_lidar_left) {

							spin_by *= (-1);
						}
					} else {
						if (a < b) {
							spin_by *= (-1);
						}
					}
					if (SimulationWindow.return_home) {
						if(!points.isEmpty()) {
							double rotation = Tools.getRotationBetweenPoints(dronePoint, getLastPoint());
							if (!(rotation >= -90 && rotation <= 90)) {
								spin_by += -1;
							}
						}
					}
					spinBy(spin_by, true, new Func() {
						@Override
						public void method() {
							try_to_escape = false;
							is_risky = false;
						}
					});
				}
			}

			
		//}
	}
	
	int counter = 0;
	
	public void doLeftRight() {
		if(is_finish) {
			leftOrRight *= -1;
			counter++;
			is_finish = false;
			
			spinBy(max_rotation_to_direction*leftOrRight,false,new Func() {
				@Override
				public void method() {
					is_finish = true;
				}
			});
		}
	}
	
	
	double lastGyroRotation = 0;
	public void updateRotating(int deltaTime) {
		
		if(degrees_left.size() == 0) {
			return;
		}
		
		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if(degrees_left_to_rotate > 0) {
			isLeft = false;
		}
		
		double curr =  drone.getGyroRotation();
		double just_rotated = 0;
		
		if(isLeft) {
			
			just_rotated = curr - lastGyroRotation;
			if(just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
		} else {
			just_rotated = curr - lastGyroRotation;
			if(just_rotated < 0) {
				just_rotated = 360 + just_rotated;
			}
		}
		
	
		 
		lastGyroRotation = curr;
		degrees_left_to_rotate-=just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0,degrees_left_to_rotate);
		
		if((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);
			
			Func func = degrees_left_func.get(0);
			if(func != null) {
				func.method();
			}
			degrees_left_func.remove(0);
			
			
			if(degrees_left.size() == 0) {
				isRotating = 0;
			}
			return; 
		}
		
		int direction = (int)(degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);
		
	}
	
	public void spinBy(double degrees,boolean isFirst,Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,func);
		
			
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}
		
		isRotating =1;
	}
	
	public void spinBy(double degrees,boolean isFirst) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,null);
		
			
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(null);
		}
		
		isRotating =1;
	}
	
	public void spinBy(double degrees) {
		lastGyroRotation = drone.getGyroRotation();
		
		degrees_left.add(degrees);
		degrees_left_func.add(null);
		isRotating = 1;
	}
	
	public Point getLastPoint() {
		if(points.size() == 0) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		return p1;
	}
	
	public Point removeLastPoint() {
		if(points.isEmpty()) {
			return init_point;
		}
		
		return points.remove(points.size()-1);
	}
	
	
	public Point getAvgLastPoint() {
		if(points.size() < 2) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		Point p2 = points.get(points.size()-2);
		return new Point((p1.x + p2.x) /2, (p1.y + p2.y) /2);
	}

	public long getTime(){
		long finish = System.currentTimeMillis();

		return (int)((finish - start)/1000)/60;


	}
	

}
