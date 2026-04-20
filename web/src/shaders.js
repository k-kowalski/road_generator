window.RoadDemoShaders = {
  lines: `
struct SceneUniforms {
  mvp: mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> uniforms: SceneUniforms;

struct VsIn {
  @location(0) position: vec3<f32>,
  @location(1) color: vec3<f32>,
};

struct VsOut {
  @builtin(position) position: vec4<f32>,
  @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(input: VsIn) -> VsOut {
  var output: VsOut;
  output.position = uniforms.mvp * vec4<f32>(input.position, 1.0);
  output.color = input.color;
  return output;
}

@fragment
fn fs_main(input: VsOut) -> @location(0) vec4<f32> {
  return vec4<f32>(input.color, 1.0);
}
`,
  ribbon: `
struct SceneUniforms {
  mvp: mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> uniforms: SceneUniforms;

struct VsIn {
  @location(0) position: vec3<f32>,
  @location(1) uv: vec2<f32>,
  @location(2) surface_kind: f32,
};

struct VsOut {
  @builtin(position) position: vec4<f32>,
  @location(0) uv: vec2<f32>,
  @location(1) @interpolate(flat) surface_kind: f32,
  @location(2) world_position: vec3<f32>,
};

@vertex
fn vs_main(input: VsIn) -> VsOut {
  var output: VsOut;
  output.position = uniforms.mvp * vec4<f32>(input.position, 1.0);
  output.uv = input.uv;
  output.surface_kind = input.surface_kind;
  output.world_position = input.position;
  return output;
}

fn saturate(value: f32) -> f32 {
  return clamp(value, 0.0, 1.0);
}

fn saturate3(value: vec3<f32>) -> vec3<f32> {
  return clamp(value, vec3<f32>(0.0), vec3<f32>(1.0));
}

fn hash21(sample_uv_in: vec2<f32>) -> f32 {
  var sample_uv = fract(sample_uv_in * vec2<f32>(0.1031, 0.1030));
  let mixed = dot(sample_uv, sample_uv.yx + vec2<f32>(33.33, 33.33));
  sample_uv = sample_uv + vec2<f32>(mixed, mixed);
  return fract((sample_uv.x + sample_uv.y) * sample_uv.x);
}

fn value_noise(sample_uv: vec2<f32>) -> f32 {
  let cell = floor(sample_uv);
  let local = fract(sample_uv);
  let smooth_local = local * local * (vec2<f32>(3.0) - 2.0 * local);

  let bottom_left = hash21(cell + vec2<f32>(0.0, 0.0));
  let bottom_right = hash21(cell + vec2<f32>(1.0, 0.0));
  let top_left = hash21(cell + vec2<f32>(0.0, 1.0));
  let top_right = hash21(cell + vec2<f32>(1.0, 1.0));

  let bottom = mix(bottom_left, bottom_right, smooth_local.x);
  let top = mix(top_left, top_right, smooth_local.x);
  return mix(bottom, top, smooth_local.y);
}

fn stripe_mask(center: f32, half_width: f32, coordinate: f32) -> f32 {
  let distance_from_center = abs(coordinate - center);
  let feather = max(fwidth(coordinate) * 0.75, 0.0025);
  return 1.0 - smoothstep(half_width - feather, half_width + feather, distance_from_center);
}

fn dash_mask(coordinate: f32, duty_cycle: f32) -> f32 {
  let repeated = fract(coordinate);
  let feather = max(fwidth(coordinate) * 1.25, 0.01);
  let begin_mask = smoothstep(0.0, feather, repeated);
  let end_mask = 1.0 - smoothstep(duty_cycle - feather, duty_cycle + feather, repeated);
  return begin_mask * end_mask;
}

fn base_asphalt(world_uv: vec2<f32>) -> vec3<f32> {
  let macro_noise = value_noise(world_uv * 0.35 + vec2<f32>(8.7, 2.3));
  let mid_noise = value_noise(world_uv * 2.4 + vec2<f32>(4.1, 9.2));
  let fine_noise = value_noise(world_uv * 10.0 + vec2<f32>(15.7, 3.4));

  var asphalt_color = mix(vec3<f32>(0.09, 0.095, 0.10), vec3<f32>(0.17, 0.175, 0.18), macro_noise);
  asphalt_color = asphalt_color + vec3<f32>((mid_noise - 0.5) * 0.025);
  asphalt_color = asphalt_color + vec3<f32>((fine_noise - 0.5) * 0.010);

  let aggregate = value_noise(world_uv * 20.0 + vec2<f32>(1.7, 6.8));
  asphalt_color = asphalt_color + vec3<f32>((aggregate - 0.5) * 0.006);
  return saturate3(asphalt_color);
}

@fragment
fn fs_main(input: VsOut) -> @location(0) vec4<f32> {
  let world_uv = input.world_position.xz;
  let road_uv = input.uv;
  let road_v = road_uv.y * 2.5;
  let intersection_mask = step(0.5, input.surface_kind);

  let asphalt_color = base_asphalt(world_uv);
  var road_color = asphalt_color;

  let lane_wear = max(
    stripe_mask(0.32, 0.10, road_uv.x),
    stripe_mask(0.68, 0.10, road_uv.x),
  );
  road_color = road_color - vec3<f32>(lane_wear * 0.04);

  let shoulder_dust = pow(abs(road_uv.x * 2.0 - 1.0), 1.6);
  road_color = road_color + shoulder_dust * vec3<f32>(0.05, 0.045, 0.03);

  var edge_paint_mask = max(
    stripe_mask(0.08, 0.022, road_uv.x),
    stripe_mask(0.92, 0.022, road_uv.x),
  );
  edge_paint_mask = edge_paint_mask *
    (0.85 + 0.15 * value_noise(vec2<f32>(road_v * 0.7, road_uv.x * 13.0)));

  var center_paint_mask = stripe_mask(0.50, 0.026, road_uv.x);
  center_paint_mask = center_paint_mask * dash_mask(road_v, 0.42);
  center_paint_mask = center_paint_mask *
    (0.55 + 0.45 * value_noise(vec2<f32>(road_v * 1.6, 4.0 + road_uv.x * 5.0)));

  let seam_glow = smoothstep(0.22, 0.0, abs(road_uv.x - 0.5));
  road_color = road_color + vec3<f32>(seam_glow * 0.01);

  let edge_paint_color = vec3<f32>(0.92, 0.90, 0.82);
  let center_paint_color = vec3<f32>(0.94, 0.72, 0.20);

  road_color = mix(road_color, edge_paint_color, saturate(edge_paint_mask));
  road_color = mix(road_color, center_paint_color, saturate(center_paint_mask));
  road_color = mix(road_color, asphalt_color, intersection_mask);

  return vec4<f32>(saturate3(road_color), 1.0);
}
`,
};
