shader_type spatial;
render_mode unshaded, cull_disabled;
uniform sampler2D tex:source_color;

void fragment() {
	ALBEDO = texture(tex,UV).rgb;
}
