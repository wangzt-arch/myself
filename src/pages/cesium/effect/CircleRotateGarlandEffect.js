import RotatingCircleEffectBase from "./RotatingCircleEffectBase";

const shaderSource = `
  uniform vec4 color;
  uniform float speed;
  uniform float iTime;
  #define PI 3.14159265359

  mat2 rotate2D(float a) {
    float s = sin(a);
    float c = cos(a);
    return mat2(c, -s, s, c);
  }

  float petal(vec2 uv, float count, float phase) {
    float angle = atan(uv.y, uv.x);
    float radius = length(uv);
    float wave = sin(angle * count + phase) * 0.5 + 0.5;
    float target = 0.42 + wave * 0.18;
    return exp(-pow((radius - target) * 13.0, 2.0)) * smoothstep(0.12, 0.35, radius);
  }

  float ring(vec2 uv, float radius, float width) {
    return 1.0 - smoothstep(width, width + 0.012, abs(length(uv) - radius));
  }

  czm_material czm_getMaterial(czm_materialInput materialInput) {
    czm_material material = czm_getDefaultMaterial(materialInput);
    vec2 uv = materialInput.st * 2.0 - 1.0;
    float time = iTime * speed;
    vec2 ruv = rotate2D(time * 0.36) * uv;
    vec2 cruv = rotate2D(-time * 0.24) * uv;

    float outer = petal(ruv, 12.0, time * 2.2);
    float inner = petal(cruv * 1.45, 9.0, -time * 1.6) * 0.7;
    float trims = ring(uv, 0.38 + sin(time) * 0.015, 0.012) + ring(uv, 0.72, 0.014);
    float center = 1.0 - smoothstep(0.08, 0.12, length(uv));
    float alpha = clamp((outer + inner + trims * 0.75 + center * 0.55) * (1.0 - smoothstep(0.82, 1.0, length(uv))), 0.0, 1.0);

    vec3 warm = vec3(1.0, 0.62, 0.16);
    vec3 tint = mix(color.rgb, warm, 0.45 + 0.25 * sin(time + length(uv) * 10.0));
    material.diffuse = tint * alpha * 1.25;
    material.emission = tint * alpha * 1.7;
    material.alpha = alpha * color.a;
    return material;
  }
`;

class CircleRotateGarlandEffect extends RotatingCircleEffectBase {
  constructor(viewer, options = {}) {
    super(viewer, options, {
      name: "CircleRotateGarlandEffect",
      materialType: "CircleRotateGarlandMaterial",
      defaultColor: "#ff9d1c",
      shaderSource,
    });
  }
}

export default CircleRotateGarlandEffect;
