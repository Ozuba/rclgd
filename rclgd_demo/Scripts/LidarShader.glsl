#[compute]
#version 450
#extension GL_EXT_nonuniform_qualifier : require


// Local size must match GDScript dispatch groups (8x8)
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// Binding 0: Scene Depth Sampler (Read-Only)
layout(set=0, binding=0) uniform sampler2D depth_buffer;


// Binding 1: Color Sampler (Read-Only)
layout(set=0, binding=1) uniform sampler2D color_buffer;

// Binding 2: Output Image
layout(set=0, binding=2, rgba32f) uniform restrict writeonly image2D lidar_out;

// Binding 2: Parameter Storage Buffer (std430 alignment is critical)
layout(set=0, binding=3, std430) restrict buffer Params {
    vec2 resolution;        // 2 floats
    float padding[2];       // 2 floats (Ensures mat4 starts on a 16-byte boundary)
    mat4 inv_proj;          // 16 floats (M_P^(-1))
    mat4 inv_view;          // 16 floats (M_VW)
} params;


void main() {

    // Determine the current pixel coordinates
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    vec2 screen_uv = vec2(xy) / params.resolution;

    // Check boundaries
    if (screen_uv.x >= 1.0 || screen_uv.y >= 1.0) {
        return;
    }

    // Sample Raw Depth (0.0 to 1.0, non-linear)
    float depth = -texture(depth_buffer, screen_uv).r;

    // Sample color_buffer (0.0 to 1.0, non-linear)
    vec4 color = texture(color_buffer, screen_uv);
    //Calculate reflectivity as a mean of color intensities
    float reflectivity = (color.r + color.g + color.b) / 3.0;


    // Convert to linear space with unprojection matrix
    vec3 ndc = vec3(screen_uv * 2.0 - 1.0,depth);
    vec4 view = params.inv_proj*vec4(ndc,1.0);
    view.xyz /= view.w;
    float linear_depth = -view.z;

    //Compute camera relative 3d world position
    vec4 world =  params.inv_proj * vec4(ndc,1.0);
    vec3 world_point = world.xyz / world.w;

    //Calculate intensity by ponderating with raw z buffer
    float intensity = clamp(reflectivity, 0.0, 1.0);

    //Copy data to output buffer
    imageStore(lidar_out, xy, vec4(-world_point, intensity));

}