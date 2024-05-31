class RadarVelocityDetectionNode(Node): 
	def __init__(self): 
		super().__init__('radar_velocity_detection_node') 
		self.subscription = self.create_subscription( 
		PointCloud2, 
		'/RADAR_TOP', 
		self.listener_callback,
		 10)
		self.publisher_ = self.create_publisher(Float32, 'velocity', 10) 
		self.get_logger().info('Radar Velocity Detection Node has been started') 
	def listener_callback(self, msg): 
		points_array = ros2_numpy.point_cloud2.pointcloud2_to_array(msg) 
		velocities = self.detect_velocities(points_array) 
		avg_velocity = np.mean(velocities) if velocities.size > 0 else 0.0 
		self.publisher_.publish(Float32(data=avg_velocity)) 
		self.get_logger().info(f'Average velocity: {avg_velocity:.2f} m/s') 
	
	def detect_velocities(self, points_array): 
	# Assuming velocity is in fields 'vx' and 'vy' 
		velocities = np.sqrt(points_array['vx']**2 + points_array['vy']**2) 
		return velocities