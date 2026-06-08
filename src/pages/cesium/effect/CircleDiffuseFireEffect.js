import * as Cesium from "cesium";

class CircleDiffuseFireEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleDiffuseFireEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 300000,
      speed: options.speed ?? 3.0,
      color: options.color,
      autoAnimate: options.autoAnimate !== false,
    };

    this.material = null;
    this.primitive = null;
    this.position = null;
    this.animationListener = null;
    this.isDestroyed = false;

    this._init();
  }

  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform float speed;
      uniform float iTime;
      vec2 iResolution = vec2(0.0);

      float rand(vec2 n) {
        return fract(sin(dot(n, vec2(12.9898, 78.233))) * 43758.5453);
      }

      float noise(vec2 p) {
        vec2 i = floor(p);
        vec2 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        return mix(
          mix(rand(i), rand(i + vec2(1.0, 0.0)), f.x),
          mix(rand(i + vec2(0.0, 1.0)), rand(i + vec2(1.0, 1.0)), f.x),
          f.y
        );
      }

      float fbm(vec2 p) {
        float value = 0.0;
        float amplitude = 0.5;
        for (int i = 0; i < 5; i++) {
          value += amplitude * noise(p);
          p *= 2.0;
          amplitude *= 0.5;
        }
        return value;
      }

      vec3 noiseImage(vec2 uv) {
        return vec3(fbm(uv * 10.0));
      }

      float snoise(vec3 uv, float res) {
        const vec3 s = vec3(1e0, 1e2, 1e4);
        uv *= res;
        vec3 uv0 = floor(mod(uv, res)) * s;
        vec3 uv1 = floor(mod(uv + vec3(1.0), res)) * s;
        vec3 f = fract(uv);
        f = f * f * (3.0 - 2.0 * f);
        vec4 v = vec4(uv0.x + uv0.y + uv0.z, uv1.x + uv0.y + uv0.z, uv0.x + uv1.y + uv0.z, uv1.x + uv1.y + uv0.z);
        vec4 r = fract(sin(v * 1e-3) * 1e5);
        float r0 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
        r = fract(sin((v + uv1.z - uv0.z) * 1e-3) * 1e5);
        float r1 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
        return mix(r0, r1, f.z) * 2.0 - 1.0;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        vec2 uv = materialInput.st * 2.0 - 1.0;
        iResolution = czm_viewport.zw;
        uv *= 0.4;

        float brightness = 0.1;
        float radius = 0.24 + brightness * 0.2;
        float invRadius = 1.0 / radius;
        float time = iTime * speed * 0.1;
        vec2 p = uv;
        float fade = pow(length(2.0 * p), 0.5);
        float fVal = 1.0 - fade;
        float angle = atan(p.x, p.y) / 3.14;
        float dist = length(p);
        vec3 coord = vec3(angle, dist, time * 0.1);
        float corona = pow(fVal * max(1.1 - fade, 0.0), 2.0) * 50.0;
        corona += pow(fVal * max(1.1 - fade, 0.0), 2.0) * 50.0;
        corona *= 1.2 - abs(snoise(coord + vec3(0.0, -time * (0.35 + brightness * 0.001), time * 0.015), 15.0));

        vec3 starSphere = vec3(0.0);
        vec2 sp = uv;
        sp *= 2.0 - brightness;
        float r = dot(sp, sp);
        float f = (1.0 - sqrt(abs(1.0 - r))) / max(r, 0.0001) + brightness * 0.5;

        if (dist < radius) {
          corona *= pow(dist * invRadius, 8.0);
          vec2 newUv = vec2(sp.x * f + time, sp.y * f);
          vec3 texSample = noiseImage(newUv);
          float uOff = texSample.g * brightness * 4.5 + time;
          vec2 starUV = newUv + vec2(uOff, 0.0);
          starSphere = noiseImage(starUV);
        }

        vec3 col = color.rgb * (corona + starSphere * 0.35);
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col.rgb));
        vec4 fragColor = vec4(col.rgb, alpha);

        material.diffuse = fragColor.rgb;
        material.emission = fragColor.rgb;
        material.alpha = fragColor.a * color.a;
        return material;
      }
    `;
  }

  _init() {
    this._registerMaterial();
    this._createPrimitive();
    if (this.options.autoAnimate) {
      this.startAnimation();
    }
  }

  _registerMaterial() {
    const CesiumRef = this._getCesium();
    const materialType = `CircleDiffuseFireMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff6a18"),
      speed: this.options.speed,
      iTime: 0.0,
    };

    if (CesiumRef.Material._materialCache) {
      CesiumRef.Material._materialCache.addMaterial(materialType, {
        fabric: {
          type: materialType,
          uniforms,
          source: this._getShaderSource(),
        },
        translucent: true,
      });
    }

    this.material = new CesiumRef.Material({
      fabric: {
        type: materialType,
        uniforms: { ...uniforms },
      },
      translucent: true,
    });
  }

  _createPrimitive() {
    const CesiumRef = this._getCesium();
    const { longitude, latitude, height, radius } = this.options;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: new CesiumRef.EllipseGeometry({
          center: this.position,
          semiMinorAxis: radius,
          semiMajorAxis: radius,
          height,
          vertexFormat: CesiumRef.EllipsoidSurfaceAppearance.VERTEX_FORMAT,
        }),
      }),
      appearance: new CesiumRef.EllipsoidSurfaceAppearance({
        material: this.material,
        aboveGround: true,
      }),
      asynchronous: false,
      show: true,
    });

    this.viewer.scene.primitives.add(this.primitive);
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("CircleDiffuseFireEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;

    const startTime = Date.now();
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed || !this.material) return;
      this.material.uniforms.iTime = (Date.now() - startTime) / 1000;
    });
  }

  stopAnimation() {
    if (this.animationListener) {
      this.viewer.scene.preRender.removeEventListener(this.animationListener);
      this.animationListener = null;
    }
  }

  destroy() {
    this.isDestroyed = true;
    this.stopAnimation();

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive);
      this.primitive = null;
    }

    if (this.material) {
      this.material.destroy();
      this.material = null;
    }

    this.viewer = null;
  }
}

export default CircleDiffuseFireEffect;
