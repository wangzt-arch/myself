import * as Cesium from "cesium";
import explosionNoise from "./explosion_noise.png";

/**
 * CircleBlastEffect - circular volumetric explosion burst.
 *
 * Adapted from the referenced CircleBlast primitive: screen-space raymarching,
 * pixel-radius scaling, and a timed blast cycle.
 */
class CircleBlastEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleBlastEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 220,
      speed: options.speed ?? 1.6,
      durationTime: options.durationTime ?? 2.0,
      color: options.color,
      autoAnimate: options.autoAnimate !== false,
    };

    this.material = null;
    this.primitive = null;
    this.position = null;
    this.animationListener = null;
    this.startTime = 0;
    this.isDestroyed = false;

    this._init();
  }

  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform float speed;
      uniform vec2 px;
      uniform float pixel;
      uniform sampler2D iChannel0;
      uniform float iTime;
      uniform float durationTime;
      vec2 iResolution = vec2(0.0);

      float hash(float n) {
        return fract(cos(n) * 41415.92653);
      }

      float hash13(vec3 p3) {
        p3 = fract(p3 * vec3(0.1031, 0.11369, 0.13787));
        p3 += dot(p3, p3.yzx + 19.19);
        return fract((p3.x + p3.y) * p3.z);
      }

      float noise(vec3 x) {
        vec3 f = fract(x);
        vec3 p = x - f;
        f = f * f * (3.0 - 2.0 * f);
        float n = p.x + p.y * 157.0 + 113.0 * p.z;
        return mix(
          mix(
            mix(hash(n + 0.0), hash(n + 1.0), f.x),
            mix(hash(n + 157.0), hash(n + 158.0), f.x),
            f.y
          ),
          mix(
            mix(hash(n + 113.0), hash(n + 114.0), f.x),
            mix(hash(n + 270.0), hash(n + 271.0), f.x),
            f.y
          ),
          f.z
        );
      }

      float fbm(vec3 p, vec3 dir) {
        vec3 q = p - dir;
        float f = 0.50000 * noise(q);
        q = q * 2.02 - dir;
        f += 0.25000 * noise(q);
        q = q * 2.03 - dir;
        f += 0.12500 * noise(q);
        q = q * 2.01 - dir;
        f += 0.06250 * noise(q);
        q = q * 2.02 - dir;
        f += 0.03125 * noise(q);
        return f;
      }

      vec2 iSphere(vec3 ro, vec3 rd, float rad) {
        float b = dot(ro, rd);
        float c = dot(ro, ro) - rad * rad;
        float h = b * b - c;
        if (h < 0.0) return vec2(-1.0);
        h = sqrt(h);
        return vec2(-b - h, -b + h);
      }

      mat3 setCamera(vec3 ro, vec3 ta) {
        vec3 cw = normalize(ta - ro);
        vec3 cp = vec3(0.0, 1.0, 0.0);
        vec3 cu = normalize(cross(cw, cp));
        vec3 cv = normalize(cross(cu, cw));
        return mat3(cu, cv, cw);
      }

      float blastDensity(vec3 p, float t) {
        float r = length(p);
        float expand = smoothstep(0.0, 0.28, t) * (1.0 - smoothstep(0.82, 1.0, t));
        float shellRadius = mix(0.18, 1.45, pow(t, 0.62));
        float shell = 1.0 - smoothstep(0.0, 0.34 + t * 0.18, abs(r - shellRadius));
        vec3 rollDir = normalize(vec3(0.15, 0.7, 0.25)) * (2.0 / (sin(min(t * 3.0, 1.57)) + 0.3));
        float warped = fbm(-2.0 * p / max(dot(p, p) * 1.25, 0.08), rollDir);
        float sparks = noise(p * 8.0 + vec3(0.0, -t * 4.0, t * 2.0));
        float density = shell * (0.35 + warped * 1.45 + sparks * 0.18);
        density *= expand;
        density *= 1.0 - smoothstep(1.25, 1.75, r);
        return clamp(density, 0.0, 1.0);
      }

      vec4 blastColor(float den, float r, float t) {
        vec3 hot = vec3(1.2, 0.94, 0.42);
        vec3 ember = color.rgb;
        vec3 smoke = vec3(0.52, 0.49, 0.44);
        float heat = smoothstep(0.56, 0.0, t) * (1.0 - smoothstep(0.55, 1.35, r));
        vec3 rgb = mix(smoke, ember, smoothstep(0.05, 0.8, den));
        rgb = mix(rgb, hot, heat * smoothstep(0.2, 0.9, den) * 0.75);
        float alpha = den * (0.45 + heat * 0.7) * (1.0 - smoothstep(0.42, 0.56, t) * 0.28);
        return vec4(rgb * alpha, alpha);
      }

      float smokeDensity(vec3 p, float t) {
        float r = length(p);
        float plumeRadius = mix(0.95, 1.75, t);
        float shell = 1.0 - smoothstep(0.0, 0.62, abs(r - plumeRadius));
        float core = 1.0 - smoothstep(0.0, plumeRadius, r);
        vec3 drift = vec3(0.0, -t * 1.8, t * 0.7);
        float rolling = fbm(p * mix(1.65, 2.45, t) + drift, vec3(0.25, 0.45, 0.12) * (1.0 + t));
        float den = (shell * 0.55 + core * 0.36) * (0.42 + rolling * 1.25);
        den *= 1.0 - smoothstep(0.74, 1.0, t) * 0.55;
        den *= 1.0 - smoothstep(1.45, 1.9, r);
        return clamp(den, 0.0, 1.0);
      }

      vec4 smokeColor(float den, float r, float t) {
        vec3 softSmoke = vec3(0.48, 0.47, 0.44);
        vec3 warmSmoke = vec3(0.62, 0.6, 0.56);
        vec3 ash = vec3(0.78, 0.77, 0.73);
        vec3 rgb = mix(softSmoke, warmSmoke, smoothstep(0.04, 0.58, den));
        rgb = mix(rgb, ash, smoothstep(0.1, 1.0, t) * smoothstep(0.68, 1.72, r) * 0.7);
        float alpha = den * mix(0.38, 0.2, t);
        return vec4(rgb * alpha, alpha);
      }

      vec4 raymarch(vec3 ro, vec3 rd, vec2 hit, float progress) {
        vec4 sum = vec4(0.0);
        float stepSize = 1.5 / 9.0;
        float marchPos = hit.x;
        vec3 pos = ro + rd * marchPos;
        float smokePhase = smoothstep(0.34, 0.68, progress);
        float smokeT = clamp((progress - 0.5) * 2.0, 0.0, 1.0);
        float blastT = min(progress, 0.5);

        for (int i = 0; i < 9; i++) {
          if (sum.a >= 0.98 || marchPos >= hit.y) break;
          float r = length(pos);
          float blastDen = blastDensity(pos, blastT);
          float grayDen = smokeDensity(pos, smokeT);
          float den = mix(blastDen, grayDen, smokePhase);
          if (den > 0.01) {
            vec4 blastCol = blastColor(blastDen, r, blastT);
            vec4 smokeCol = smokeColor(grayDen, r, smokeT);
            vec4 col = mix(blastCol, smokeCol, smokePhase);
            col.rgb = mix(blastCol.rgb, smokeCol.rgb, smokePhase * smokePhase * (3.0 - 2.0 * smokePhase));
            col.a = mix(blastCol.a, smokeCol.a, smoothstep(0.0, 1.0, smokePhase));
            sum = sum + col * (1.0 - sum.a);
            sum.a += 0.12 * col.a;
          }
          float stepMult = 1.0 + (1.0 - clamp(den + sum.a, 0.0, 1.0));
          pos += rd * stepSize * stepMult;
          marchPos += stepSize * stepMult;
        }

        return clamp(sum, 0.0, 1.0);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        iResolution = czm_viewport.zw;
        vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        vec2 center = vec2(px.x, iResolution.y - px.y);
        vec2 p2 = (2.0 * center.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        uv = uv - p2;
        uv = uv * 1.5 * min(iResolution.x, iResolution.y) / pixel;

        float cycle = max(durationTime, 0.1);
        float t = clamp(iTime * speed / (cycle * 2.0), 0.0, 1.0);
        float camAngle = iTime * 11.7;
        vec3 ro = vec3(cos(camAngle) * 5.0, sin(camAngle * 0.014) * 0.75, sin(camAngle) * 5.0);
        vec3 rd = setCamera(ro, vec3(0.0)) * normalize(vec3(uv.xy * 0.5, 1.0));
        vec2 hit = iSphere(ro, rd, 1.75);
        vec4 col = vec4(0.0);

        if (hit.x > 0.0) {
          col = raymarch(ro, rd, hit, t);
        }

        col.rgb = mix(vec3(0.64, 0.61, 0.56), col.rgb, smoothstep(0.0, 0.28, col.a));
        col.rgb = max(col.rgb, vec3(0.42, 0.4, 0.37) * col.a);
        material.diffuse = col.rgb;
        material.alpha = col.a * color.a;
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
    const materialType = `CircleBlastMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || new CesiumRef.Color(1.0, 0.35, 0.08, 1.0),
      speed: this.options.speed,
      iChannel0: explosionNoise,
      px: new CesiumRef.Cartesian2(0.0, 0.0),
      pixel: this.options.radius * 2.0,
      iTime: 0.0,
      durationTime: this.options.durationTime,
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
    const { longitude, latitude, height } = this.options;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: CesiumRef.BoxGeometry.fromDimensions({
          vertexFormat: CesiumRef.EllipsoidSurfaceAppearance.VERTEX_FORMAT,
          dimensions: new CesiumRef.Cartesian3(1.0, 1.0, 1.0),
        }),
      }),
      appearance: new CesiumRef.EllipsoidSurfaceAppearance({
        material: this.material,
        aboveGround: true,
      }),
      asynchronous: false,
      show: true,
    });

    this._syncPrimitive();
    this.viewer.scene.primitives.add(this.primitive);
  }

  _syncPrimitive() {
    if (!this.primitive || !this.position || !this.material) return;

    const CesiumRef = this._getCesium();
    this.material.uniforms.px = this._getWindowCoordinates() || new CesiumRef.Cartesian2(0.0, 0.0);
    this.material.uniforms.pixel = this.options.radius * 2.0;

    const realRadius = this._getMetersPerPixel() * this.options.radius;
    const translationMatrix = CesiumRef.Transforms.eastNorthUpToFixedFrame(this.position);
    const scaleMatrix = CesiumRef.Matrix4.fromScale(new CesiumRef.Cartesian3(realRadius, realRadius, realRadius));
    this.primitive.modelMatrix = CesiumRef.Matrix4.multiply(translationMatrix, scaleMatrix, new CesiumRef.Matrix4());
  }

  _getMetersPerPixel() {
    if (!this.viewer.scene.canvas.clientWidth) return 0.0;
    const CesiumRef = this._getCesium();
    const distance = CesiumRef.Cartesian3.distance(this.viewer.camera.position, this.position);
    const metersPerPixel = distance / this.viewer.scene.canvas.clientWidth;
    return Number.isFinite(metersPerPixel) ? metersPerPixel : 0.0;
  }

  _getWindowCoordinates() {
    const CesiumRef = this._getCesium();
    if (CesiumRef.SceneTransforms.wgs84ToWindowCoordinates) {
      return CesiumRef.SceneTransforms.wgs84ToWindowCoordinates(this.viewer.scene, this.position);
    }
    if (CesiumRef.SceneTransforms.worldToWindowCoordinates) {
      return CesiumRef.SceneTransforms.worldToWindowCoordinates(this.viewer.scene, this.position);
    }
    return undefined;
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("CircleBlastEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;

    this.startTime = Date.now();
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed) return;
      const speed = Math.max(this.options.speed, 0.001);
      const totalTime = Math.max(this.options.durationTime, 0.1);
      const visibleTime = totalTime * 2.0 / speed;
      const delta = ((Date.now() - this.startTime) / 1000) % visibleTime;
      this.material.uniforms.iTime = delta;
      this.material.uniforms.durationTime = this.options.durationTime;
      this._syncPrimitive();
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

  moveTo(longitude, latitude, height) {
    const CesiumRef = this._getCesium();
    this.options.longitude = longitude;
    this.options.latitude = latitude;
    if (height !== undefined) {
      this.options.height = height;
    }
    this.position = CesiumRef.Cartesian3.fromDegrees(this.options.longitude, this.options.latitude, this.options.height);
    this._syncPrimitive();
  }

  setRadius(radius) {
    this.options.radius = radius;
    this._syncPrimitive();
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

export default CircleBlastEffect;
