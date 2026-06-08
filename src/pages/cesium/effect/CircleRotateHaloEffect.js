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

  float lineRing(vec2 uv, float radius, float width) {
    return 1.0 - smoothstep(width, width + 0.006, abs(length(uv) - radius));
  }

  float arcMask(vec2 uv, float segments, float open, float phase) {
    float angle = atan(uv.y, uv.x) / (PI * 2.0) + 0.5;
    return smoothstep(open, open + 0.04, fract(angle * segments + phase));
  }

  czm_material czm_getMaterial(czm_materialInput materialInput) {
    czm_material material = czm_getDefaultMaterial(materialInput);
    vec2 uv = materialInput.st * 2.0 - 1.0;
    float time = iTime * speed;
    float radius = length(uv);

    vec2 fast = rotate2D(time * 0.54) * uv;
    vec2 slow = rotate2D(-time * 0.21) * uv;
    float halo = lineRing(fast, 0.36, 0.018) * arcMask(fast, 6.0, 0.28, time * 0.12);
    halo += lineRing(slow, 0.58, 0.014) * arcMask(slow, 10.0, 0.42, -time * 0.09);
    halo += lineRing(uv, 0.76, 0.012) * (0.65 + 0.35 * sin(time * 2.0));
    float glow = exp(-pow((radius - 0.58) * 4.2, 2.0)) * 0.18;
    float alpha = clamp((halo + glow) * (1.0 - smoothstep(0.82, 1.0, radius)), 0.0, 1.0);

    vec3 cool = vec3(0.2, 0.86, 1.0);
    vec3 tint = mix(cool, color.rgb, 0.55);
    material.diffuse = tint * alpha * 1.45;
    material.emission = tint * alpha * 2.2;
    material.alpha = alpha * color.a;
    return material;
  }
`;

class CircleRotateHaloEffect extends RotatingCircleEffectBase {
  constructor(viewer, options = {}) {
    super(viewer, options, {
      name: "CircleRotateHaloEffect",
      materialType: "CircleRotateHaloMaterial",
      defaultColor: "#40b7ff",
      shaderSource,
    });
  }
}

export default CircleRotateHaloEffect;
