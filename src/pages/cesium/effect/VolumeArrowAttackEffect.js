import * as Cesium from "cesium";

// 万箭穿心
class VolumeArrowAttackEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("VolumeArrowAttackEffect: viewer is required");
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
      uniform vec2 px;
      uniform float pixel;

      float iTime = 0.0;
      vec2 iResolution = vec2(0.0);

      const float ARROWS = 5.0;
      const float VANISH_TIME = 1.5;
      const float PI = 3.1415926;
      const float TAU = PI * 2.0;
      const float HPI = PI * 0.5;

      float dot2(vec2 x) { return dot(x, x); }

      float hash11(float p) {
        p = fract(p * 0.1031); p *= p + 33.33;
        return fract(p * p * 2.0);
      }

      float noise(float p) {
        float fl = floor(p); float fc = fract(p);
        fc *= fc * (3.0 - 2.0 * fc);
        return mix(hash11(fl), hash11(fl + 1.0), fc);
      }

      float hash12(vec2 p) {
        vec3 p3 = fract(p.xyx * .1031);
        p3 += dot(p3, p3.yzx + 33.33);
        return fract((p3.x + p3.y) * p3.z);
      }

      float noise2d(vec2 p) {
        vec2 ip = floor(p); vec2 u = fract(p);
        u = u * u * (3.0 - 2.0 * u);
        return mix(mix(hash12(ip), hash12(ip + vec2(1, 0)), u.x),
          mix(hash12(ip + vec2(0, 1)), hash12(ip + vec2(1)), u.x), u.y);
      }

      float fbm(float p, int octaves) {
        float s = 0.0, m = 0.0, a = 1.0;
        for(int i = 0; i < 3; i++) {
          s += a * noise(p); m += a; a *= 0.6; p *= 1.8;
        }
        return s / m;
      }

      float fbm2d(vec2 p, int octaves) {
        float s = 0.0, m = 0.0, a = 1.0;
        for(int i = 0; i < 4; i++) {
          s += a * noise2d(p); m += a; a *= 0.6; p *= 1.8;
        }
        return s / m;
      }

      float shake(float t) {
        t = clamp(t, 0.0, 1.0);
        return (fbm(pow(t, 0.3) * 8.0, 3) * (1.0 + cos(t * HPI)) - sin(t * HPI)) * (1.0 - t) * pow(t, 0.7);
      }

      float arrow(vec2 uv) {
        const float outline = 0.01;
        const float line_width = 1.8;
        float height = 0.2 * max(0.5, sqrt(uv.x));
        const float head_size = 0.2;
        const float size = 0.02;
        uv.x -= line_width * 0.45 + head_size * 0.5;
        float a1 = abs(uv.y - (-head_size * (-height)));
        float a2 = abs(uv.y - 0.0);
        float head = min(max(a1, a2), max(abs(uv.y - head_size * height), a2));
        uv.y = abs(uv.y);
        float tail = abs(uv.y - 0.0);
        float stick = min(tail, abs(uv.x + 0.2));
        return min(abs(head - size) - outline, stick - outline);
      }

      float shoot_arrow(vec2 uv, float t, float id) {
        t = clamp(t - id, 0.0, 1.0);
        float f = length(fwidth(uv));
        vec2 auv = uv;
        float a = noise(-id * 5.0 - 0.05);
        auv = cos(a * TAU) * auv + sin(a * TAU) * vec2(-auv.y, auv.x);
        auv.x -= (smoothstep(0.0, 0.8, t * 2.0) - 1.0) * 7.0;
        return arrow(auv);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        iTime = czm_frameNumber/60.0 * speed;
        iResolution = czm_viewport.zw;

        vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / min(iResolution.x,iResolution.y);
        vec2 center = vec2(px.x, iResolution.y - px.y);
        vec2 p2 = (2.0 * center.xy - iResolution.xy) / min(iResolution.x,iResolution.y);
        uv = uv - p2;
        uv = uv * 2.0 * min(iResolution.x,iResolution.y)/pixel;

        float f = length(fwidth(uv));
        float t = mod(iTime * 2.0, ARROWS + VANISH_TIME);
        float ft = floor(t);
        float a = noise(-ft * 5.0 - 0.05);
        vec2 dir = vec2(cos(-a * TAU), sin(-a * TAU));
        float ts = fract(min(t, ARROWS));
        float s = shake((ts * 2.0 - 0.65) * 2.5) * 0.7;
        uv -= dir * s;

        vec3 col = color.rgb;
        float arrw = shoot_arrow(uv, t, 0.0);
        for (float i = 1.0; i < ARROWS; ++i)
          arrw = min(arrw, shoot_arrow(uv, t, i));
        arrw += fbm2d(uv * 4.0, 4) * smoothstep(ARROWS, ARROWS + VANISH_TIME, t) * 0.5;
        col *= vec3(smoothstep(f, -f, arrw));
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0,1.0,length(col.rgb));
        vec4 fragColor = vec4(col.rgb, alpha);

        material.diffuse = fragColor.rgb * 2.0;
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
    const materialType = `VolumeArrowAttackMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff3333").withAlpha(0.9),
      speed: this.options.speed,
      px: new CesiumRef.Cartesian2(0.0, 0.0),
      pixel: 300.0,
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
    throw new Error("VolumeArrowAttackEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed || !this.material) return;
      const CesiumRef = this._getCesium();
      let px;
      if (CesiumRef.SceneTransforms.wgs84ToWindowCoordinates) {
        px = CesiumRef.SceneTransforms.wgs84ToWindowCoordinates(this.viewer.scene, this.position);
      } else if (CesiumRef.SceneTransforms.worldToWindowCoordinates) {
        px = CesiumRef.SceneTransforms.worldToWindowCoordinates(this.viewer.scene, this.position);
      }
      if (px) {
        this.material.uniforms.px = px;
      }
    });
  }

  stopAnimation() {
    if (this.animationListener) {
      this.viewer.scene.preRender.removeEventListener(this.animationListener);
      this.animationListener = null;
    }
  }

  setSpeed(speed) {
    this.options.speed = speed;
    if (this.material) {
      this.material.uniforms.speed = speed;
    }
  }

  setColor(color) {
    if (this.material) {
      this.material.uniforms.color = color;
    }
  }

  show() {
    if (this.primitive) {
      this.primitive.show = true;
    }
  }

  hide() {
    if (this.primitive) {
      this.primitive.show = false;
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

export default VolumeArrowAttackEffect;
