extends SubViewport

const TEXTURE_SIZE = Vector2(512, 512)
@export var displayPlane: MeshInstance3D 

var texture_rid: RID = RID()
var rd: RenderingDevice
var ros_node: RosNode
var lidar_pub: RosPublisher

var cached_msg: RosMsg
var cached_fields: Array = []
var is_sampling: bool = false # PREVENT OVERLAP

func _ready() -> void:
	if not rclgd.ok():
		rclgd.init([])
	ros_node = RosNode.new()
	ros_node.init("GPULidar")
	lidar_pub = ros_node.create_publisher("/gpu_lidar", "sensor_msgs/msg/PointCloud2")
	
	rd = RenderingServer.get_rendering_device()
	if not rd: return

	var fmt : RDTextureFormat = RDTextureFormat.new()
	fmt.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT
	fmt.width = TEXTURE_SIZE.x
	fmt.height = TEXTURE_SIZE.y
	fmt.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | \
					 RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | \
					 RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	
	texture_rid = rd.texture_create(fmt, RDTextureView.new())
	
	var effect_resource = $LidarCamera.compositor.compositor_effects[0]
	effect_resource.target_tex = texture_rid
	effect_resource.texture_size = TEXTURE_SIZE

	_prepare_msg_template()
	
	if is_instance_valid(displayPlane):
		var tex_preview = Texture2DRD.new()
		tex_preview.texture_rd_rid = texture_rid
		displayPlane.get_active_material(0).set_shader_parameter("tex", tex_preview)

func _prepare_msg_template():
	cached_fields = [
		_create_field("x", 0, 7),
		_create_field("y", 4, 7),
		_create_field("z", 8, 7),
		_create_field("intensity", 12, 7)
	]
	#cached_msg = RosMsg.from_type("sensor_msgs/msg/PointCloud2")
	cached_msg = RosSensorMsgsPointCloud2.new()
	cached_msg.height = 1
	cached_msg.is_dense = true
	cached_msg.point_step = 16 
	cached_msg.fields = cached_fields
	cached_msg.header.frame_id = "map"
	

func _create_field(name: String, offset: int, datatype: int) -> RosMsg:
	var f = RosSensorMsgsPointField.new()
	f.name = name
	f.offset = offset
	f.datatype = datatype
	f.count = 1
	return f

# Request a frame only if we aren't already waiting for one
func _process(_delta):
	if not is_sampling:
		sample()

func sample():
	if not rd or not texture_rid.is_valid(): return
	is_sampling = true
	rd.texture_get_data_async(texture_rid, 0, _on_texture_data_ready)

func _on_texture_data_ready(raw_bytes: PackedByteArray):
	# Release the lock immediately so the next _process can queue a request
	is_sampling = false
	if raw_bytes.is_empty():
		return 
	
	# Only update dynamic data
	cached_msg.width = raw_bytes.size() / 16
	cached_msg.row_step = raw_bytes.size()
	cached_msg.data = raw_bytes 
	cached_msg.header.stamp = ros_node.now() 
	lidar_pub.publish(cached_msg)
