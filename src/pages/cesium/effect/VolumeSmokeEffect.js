import * as Cesium from "cesium";
import smokeNoise from "./smoke_noise.png";

/**
 * VolumeSmokeEffect - Shadertoy-style volumetric smoke primitive. [trigger-rebuild]
 *
 * Ported from the referenced VolumeSmoke implementation: it renders an
 * ellipsoid with a screen-space raymarching material and keeps its world size
 * tied to a pixel radius, so the smoke reads consistently while zooming.
 * 体积烟雾效果 - 类似Shadertoy的体积烟雾原语。
 */
class VolumeSmokeEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("VolumeSmokeEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 170,
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
      uniform sampler2D iChannel0;
      float iTime = 0.0;
      vec2 iResolution = vec2(0.0);

      vec4 sphere = vec4(0.0, 0.0, 0.0, 1.0);

      float hash(float p) {
        p = fract(p * 0.011);
        p *= p + 7.5;
        p *= p + p;
        return fract(p);
      }

      float hash(vec2 p) {
        vec3 p3 = fract(vec3(p.xyx) * 0.13);
        p3 += dot(p3, p3.yzx + 3.333);
        return fract((p3.x + p3.y) * p3.z);
      }

      float noise(float x) {
        float i = floor(x);
        float f = fract(x);
        float u = f * f * (3.0 - 2.0 * f);
        return mix(hash(i), hash(i + 1.0), u);
      }

      float noise(vec2 x) {
        vec2 i = floor(x);
        vec2 f = fract(x);
        float a = hash(i);
        float b = hash(i + vec2(1.0, 0.0));
        float c = hash(i + vec2(0.0, 1.0));
        float d = hash(i + vec2(1.0, 1.0));
        vec2 u = f * f * (3.0 - 2.0 * f);
        return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
      }

      float noise(vec3 x) {
        const vec3 step = vec3(110.0, 241.0, 171.0);
        vec3 i = floor(x);
        vec3 f = fract(x);
        float n = dot(i, step);
        vec3 u = f * f * (3.0 - 2.0 * f);
        return mix(
          mix(
            mix(hash(n + dot(step, vec3(0.0, 0.0, 0.0))), hash(n + dot(step, vec3(1.0, 0.0, 0.0))), u.x),
            mix(hash(n + dot(step, vec3(0.0, 1.0, 0.0))), hash(n + dot(step, vec3(1.0, 1.0, 0.0))), u.x),
            u.y
          ),
          mix(
            mix(hash(n + dot(step, vec3(0.0, 0.0, 1.0))), hash(n + dot(step, vec3(1.0, 0.0, 1.0))), u.x),
            mix(hash(n + dot(step, vec3(0.0, 1.0, 1.0))), hash(n + dot(step, vec3(1.0, 1.0, 1.0))), u.x),
            u.y
          ),
          u.z
        );
      }

      float fractal_noise(vec3 p) {
        float f = 0.0;
        p = p - vec3(1.0, 1.0, 0.0) * iTime * 0.1;
        p = p * 3.0;
        f += 0.50000 * noise(p); p = 2.0 * p;
        f += 0.25000 * noise(p); p = 2.0 * p;
        f += 0.12500 * noise(p); p = 2.0 * p;
        f += 0.06250 * noise(p); p = 2.0 * p;
        f += 0.03125 * noise(p);
        return f;
      }

      float sphIntersect(vec3 ro, vec3 rd, vec4 sph) {
        vec3 oc = ro - sph.xyz;
        float b = dot(oc, rd);
        float c = dot(oc, oc) - sph.w * sph.w;
        float h = b * b - c;
        if (h < 0.0) return -1.0;
        h = sqrt(h);
        return -b - h;
      }

      float density(vec3 pos, float dist) {
        float den = -0.2 - dist * 1.5 + 3.0 * fractal_noise(pos);
        den = clamp(den, 0.0, 1.0);
        float size = clamp(texture(iChannel0, vec2(0.5, 0.0)).x * 2.0 + 0.1, 0.4, 0.8);
        float edge = 1.0 - smoothstep(size * sphere.w, sphere.w, dist);
        edge *= edge;
        den *= edge;
        return den;
      }

      vec3 colorFn(float den, float dist) {
        vec3 result = mix(
          vec3(1.0, 0.9, 0.8 + sin(iTime) * 0.1),
          vec3(0.5, 0.15, 0.1 + sin(iTime) * 0.1),
          den * den
        );
        vec3 colBot = 3.0 * vec3(1.0, 0.9, 0.5);
        vec3 colTop = 2.0 * vec3(0.5, 0.55, 0.55);
        result *= mix(colBot, colTop, min((dist + 0.5) / sphere.w, 1.0));
        return result;
      }

      vec3 raymarching(vec3 ro, vec3 rd, float t, vec3 backCol) {
        vec4 sum = vec4(0.0);
        vec3 pos = ro + rd * t;
        for (int i = 0; i < 30; i++) {
          float dist = length(pos - sphere.xyz);
          if (dist > sphere.w + 0.01 || sum.a > 0.99) break;

          float den = density(pos, dist);
          vec4 col = vec4(colorFn(den, dist), den);
          col.rgb *= col.a;
          sum = sum + col * (1.0 - sum.a);

          t += max(0.05, 0.02 * t);
          pos = ro + rd * t;
        }

        sum = clamp(sum, 0.0, 1.0);
        return mix(backCol, sum.xyz, sum.a);
      }

      mat3 setCamera(vec3 ro, vec3 ta, float cr) {
        vec3 cw = normalize(ta - ro);
        vec3 cp = vec3(sin(cr), cos(cr), 0.0);
        vec3 cu = normalize(cross(cw, cp));
        vec3 cv = normalize(cross(cu, cw));
        return mat3(cu, cv, cw);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        iTime = czm_frameNumber / 60.0 * speed;
        iResolution = czm_viewport.zw;
        vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        vec2 center = vec2(px.x, iResolution.y - px.y);
        vec2 p2 = (2.0 * center.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        uv = uv - p2;
        uv = uv * 1.2 * min(iResolution.x, iResolution.y) / pixel;

        vec2 p = uv;
        vec2 mo = vec2(iTime * 0.5);
        vec3 ro = vec3(0.0, 0.0, -2.0);
        vec2 cossin = vec2(cos(mo.x), sin(mo.x));
        mat3 rot = mat3(
          cossin.x, 0.0, -cossin.y,
          0.0, 1.0, 0.0,
          cossin.y, 0.0, cossin.x
        );
        ro = rot * ro;
        cossin = vec2(cos(mo.y), sin(mo.y));
        rot = mat3(
          1.0, 0.0, 0.0,
          0.0, cossin.x, -cossin.y,
          0.0, cossin.y, cossin.x
        );
        ro = rot * ro;

        vec3 rd = setCamera(ro, vec3(0.0), 0.0) * normalize(vec3(p.xy, 1.5));
        float dist = sphIntersect(ro, rd, sphere);
        vec3 col = vec3(0.45, 0.4, 0.4) * (1.0 - 0.3 * length(p));

        if (dist > 0.0) {
          col = raymarching(ro, rd, dist, col);
        }

        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col.rgb));
        alpha = pow(alpha, 6.0);
        vec4 fragColor = vec4(col.rgb, alpha);

        material.diffuse = fragColor.rgb;
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
    const materialType = `VolumeSmokeMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || new CesiumRef.Color(0.65, 0.62, 0.58, 0.82),
      speed: this.options.speed,
      iChannel0: smokeNoise,
      px: new CesiumRef.Cartesian2(0.0, 0.0),
      pixel: this.options.radius * 2.0,
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
        geometry: new CesiumRef.EllipsoidGeometry({
          radii: new CesiumRef.Cartesian3(1.0, 1.0, 1.0),
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

    this._syncPrimitive();
    this.viewer.scene.primitives.add(this.primitive);
  }

  _syncPrimitive() {
    if (!this.primitive || !this.position || !this.material) return;

    const CesiumRef = this._getCesium();
    const px = this._getWindowCoordinates();
    this.material.uniforms.px = px || new CesiumRef.Cartesian2(0.0, 0.0);
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
    if (CesiumRef.SceneTransforms && CesiumRef.SceneTransforms.worldToWindowCoordinates) {
      return CesiumRef.SceneTransforms.worldToWindowCoordinates(this.viewer.scene, this.position);
    }
    return undefined;
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("VolumeSmokeEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;

    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed) return;
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

export default VolumeSmokeEffect;
