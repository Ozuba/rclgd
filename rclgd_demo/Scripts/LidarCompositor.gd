extends CompositorEffect
class_name RenderingEffectDepthCapture

# --- Configuration (Set by the Lidar node externally) ---
var texture_size: Vector2 = Vector2(512, 512) 

# --- RIDs ---
var target_tex: RID # The texture we output to 
var rd: RenderingDevice
var nearest_sampler: RID
var shader: RID = RID()
var pipeline: RID = RID()

# --- Initialization ---

func _init() -> void:
	effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	# Start asynchronous initialization immediately upon creation.
	RenderingServer.call_on_render_thread(_initialize_compute)

func _notification(what: int) -> void:
	# Cleanup: Fires when the resource is freed. It must clean up RIDs.
	if what == NOTIFICATION_PREDELETE and rd:
		# Check if RIDs are valid before attempting to free
		if shader.is_valid():
			rd.free_rid(shader)
		if pipeline.is_valid():
			rd.free_rid(pipeline)
		if nearest_sampler.is_valid():
			rd.free_rid(nearest_sampler)

func _initialize_compute() -> void:
	rd = RenderingServer.get_rendering_device()
	if !rd:
		return
		
	# Create Sampler
	var sampler_state: RDSamplerState = RDSamplerState.new()
	sampler_state.min_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
	sampler_state.mag_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
	nearest_sampler = rd.sampler_create(sampler_state)
	
	# Create Shader and Pipeline
	var shader_file: RDShaderFile = load("res://Scripts/LidarShader.glsl")
	var shader_spirv: RDShaderSPIRV = shader_file.get_spirv()
	shader = rd.shader_create_from_spirv(shader_spirv)
	
	# Only create pipeline if shader creation succeeded
	if shader.is_valid():
		pipeline = rd.compute_pipeline_create(shader)


# --- Render Callback (The Execution Loop) ---
func _render_callback(p_effect_callback_type: int, p_render_data: RenderData) -> void:
	# 1. CRITICAL VALIDITY CHECK: Avoid race conditions from asynchronous init
	if not shader.is_valid() or not pipeline.is_valid() or not target_tex.is_valid():
		return
		
	if rd and p_effect_callback_type == CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT:
		
		var render_scene_buffers: RenderSceneBuffersRD = p_render_data.get_render_scene_buffers()
		# API FIX: Retrieve the RenderSceneData object, which holds view transforms.
		var render_scene_data: RenderSceneData = p_render_data.get_render_scene_data()
		
		if not render_scene_buffers or not render_scene_data:
			return

		var size: Vector2i = render_scene_buffers.get_internal_size()
		if size.x == 0 or size.y == 0:
			return

		# Compute Dispatch Groups (assuming local_size_x/y = 8 in the shader)
		var x_groups: int = (size.x - 1) / 8 + 1
		var y_groups: int = (size.y - 1) / 8 + 1
		
		var view_count: int = render_scene_buffers.get_view_count()
		
		for view in range(view_count):
			
			# --- API FIX: Access matrices via RenderSceneData ---
			
			# 1. Get the World-to-View Transform (View Matrix)
			var view_matrix: Transform3D = render_scene_data.get_cam_transform()
			
			# 2. Get the Projection Matrix
			var projection_matrix: Projection = render_scene_data.get_cam_projection()

			# --- Calculate Inverses ---
			var inv_view_matrix: Transform3D = view_matrix.affine_inverse()
			var inv_projection_matrix: Projection = projection_matrix.inverse()
			
			#print("--- View #", view, " Matrices ---")
			
			## Print Inverse View Matrix (Transform3D)
			#print("Inv View (Transform3D):")
			#print("  Basis X:", inv_view_matrix.basis.x)
			#print("  Basis Y:", inv_view_matrix.basis.y)
			#print("  Basis Z:", inv_view_matrix.basis.z)
			#print("  Origin: ", inv_view_matrix.origin)
			#
			## Print Inverse Projection Matrix (Projection)
			#print("Inv Projection (Projection):")
			#print("  Col X: ", inv_projection_matrix.x)
			#print("  Col Y: ", inv_projection_matrix.y)
			#print("  Col Z: ", inv_projection_matrix.z)
			#print("  Col W: ", inv_projection_matrix.w)
			
			# --- Uniform Setup (U0 and U1) ---
			var depth_tex: RID = render_scene_buffers.get_depth_layer(view)
			var color_tex: RID = render_scene_buffers.get_color_layer(view)

			# U0: Depth Sampler
			var depth_uniform: RDUniform = RDUniform.new()
			depth_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
			depth_uniform.binding = 0
			depth_uniform.add_id(nearest_sampler)
			depth_uniform.add_id(depth_tex)
			
			# U1: Albedo Sampler
			var color_uniform: RDUniform = RDUniform.new()
			color_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
			color_uniform.binding = 1
			color_uniform.add_id(nearest_sampler)
			color_uniform.add_id(color_tex)
			
			
			# U2: Output Image/Buffer
			var depth_copy_uniform: RDUniform = RDUniform.new()
			depth_copy_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
			depth_copy_uniform.binding = 2
			depth_copy_uniform.add_id(target_tex)

			# --- U2: Params Buffer Setup (Sending 34 floats: 2 + 16 + 16) ---
			# 1. Resolution
			var resolution_array = PackedFloat32Array([
				texture_size.x,
				texture_size.y,
			])

			# 2. PADDING - CRITICAL: 2 floats (8 bytes) to align the following mat4 to a 16-byte boundary
			var padding_array = PackedFloat32Array([
				0.0,
				0.0,
			])

			# 3. Inverse Projection Matrix (M_P^(-1)) - 16 floats
			# Use the correct Column-Major packing format
			var inv_proj_array = PackedFloat32Array([
				# Column 0 (X)
				inv_projection_matrix.x.x, inv_projection_matrix.x.y, inv_projection_matrix.x.z, inv_projection_matrix.x.w, 
				# Column 1 (Y)
				inv_projection_matrix.y.x, inv_projection_matrix.y.y, inv_projection_matrix.y.z, inv_projection_matrix.y.w, 
				# Column 2 (Z)
				inv_projection_matrix.z.x, inv_projection_matrix.z.y, inv_projection_matrix.z.z, inv_projection_matrix.z.w, 
				# Column 3 (W)
				inv_projection_matrix.w.x, inv_projection_matrix.w.y, inv_projection_matrix.w.z, inv_projection_matrix.w.w, 
			])

			# 4. Inverse View Matrix (M_VW) - 16 floats
			# Include the final required sign flips for X and Y Basis vectors.
			var inv_view_array = PackedFloat32Array([
				# Column 0: Basis.x (FLIPPED)
				inv_view_matrix.basis.x.x, inv_view_matrix.basis.x.y, inv_view_matrix.basis.x.z, 0.0, 
				# Column 1: Basis.y (FLIPPED)
				inv_view_matrix.basis.y.x, inv_view_matrix.basis.y.y, inv_view_matrix.basis.y.z, 0.0, 
				# Column 2: Basis.z (KEPT SAME)
				inv_view_matrix.basis.z.x, inv_view_matrix.basis.z.y, inv_view_matrix.basis.z.z, 0.0, 
				# Column 3: Origin (Translation)
				inv_view_matrix.origin.x, inv_view_matrix.origin.y, inv_view_matrix.origin.z, 1.0, 
			])


			# --- Final Concatenation ---
			var params = resolution_array + padding_array + inv_proj_array + inv_view_array
			var params_byte_array = params.to_byte_array()
			var params_buffer: RID = rd.storage_buffer_create(params_byte_array.size(), params_byte_array)
			
			
			# U3: Params Storage Buffer
			var params_uniform: RDUniform = RDUniform.new()
			params_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
			params_uniform.binding = 3
			params_uniform.add_id(params_buffer)

			# --- Dispatch ---
			var uniform_set: RID = rd.uniform_set_create([depth_uniform,color_uniform, depth_copy_uniform, params_uniform], shader, 0)
			
			var compute_list := rd.compute_list_begin()
			
			rd.compute_list_bind_compute_pipeline(compute_list, pipeline)
			rd.compute_list_bind_uniform_set(compute_list, uniform_set, 0)
			rd.compute_list_dispatch(compute_list, x_groups, y_groups, 1)
			rd.compute_list_end()

			# --- Cleanup ---
			rd.free_rid(uniform_set)
			rd.free_rid(params_buffer)
