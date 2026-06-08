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

  float ringLine(vec2 uv, float radius, float width) {
    return 1.0 - smoothstep(width, width + 0.012, abs(length(uv) - radius));
  }

  float radialLine(vec2 uv, float count, float width, float offset) {
    float angle = atan(uv.y, uv.x) / (PI * 2.0) + 0.5;
    float line = abs(fract(angle * count + offset) - 0.5);
    return 1.0 - smoothstep(width, width + 0.01, line);
  }

  czm_material czm_getMaterial(czm_materialInput materialInput) {
    czm_material material = czm_getDefaultMaterial(materialInput);
    vec2 uv = materialInput.st * 2.0 - 1.0;
    float time = iTime * speed;
    uv = rotate2D(time * 0.42) * uv;

    float radius = length(uv);
    float fade = 1.0 - smoothstep(0.82, 1.0, radius);
    float core = ringLine(uv, 0.28, 0.012) + ringLine(uv, 0.52, 0.012) + ringLine(uv, 0.76, 0.01);
    float spokes = radialLine(uv, 18.0, 0.035, sin(time) * 0.15) * smoothstep(0.18, 0.65, radius) * (1.0 - smoothstep(0.68, 0.9, radius));
    float sweep = pow(max(0.0, sin(atan(uv.y, uv.x) * 3.0 - time * 2.6)), 10.0) * smoothstep(0.2, 0.7, radius);
    float alpha = clamp((core + spokes * 0.8 + sweep * 1.4) * fade, 0.0, 1.0);

    vec3 palette = mix(vec3(0.0, 0.85, 1.0), vec3(1.0, 0.22, 0.95), sin(time + radius * 18.0) * 0.5 + 0.5);
    material.diffuse = mix(color.rgb, palette, 0.72) * alpha * 1.35;
    material.emission = material.diffuse * 1.8;
    material.alpha = alpha * color.a;
    return material;
  }
`;

class CircleRotateColorLineEffect extends RotatingCircleEffectBase {
  constructor(viewer, options = {}) {
    super(viewer, options, {
      name: "CircleRotateColorLineEffect",
      materialType: "CircleRotateColorLineMaterial",
      defaultColor: "#8b5cff",
      shaderSource,
    });
  }
}

export default CircleRotateColorLineEffect;
