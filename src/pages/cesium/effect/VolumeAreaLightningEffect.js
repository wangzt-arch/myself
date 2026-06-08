import * as Cesium from "cesium";

// 区域闪电
class VolumeAreaLightningEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("VolumeAreaLightningEffect: viewer is required");
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

      float hash(vec2 p) {
        return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
      }

      float noise(vec2 p) {
        vec2 i = floor(p);
        vec2 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        float a = hash(i);
        float b = hash(i + vec2(1.0, 0.0));
        float c = hash(i + vec2(0.0, 1.0));
        float d = hash(i + vec2(1.0, 1.0));
        return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
      }

      float fbm(vec2 p) {
        float v = 0.0;
        float a = 0.5;
        for (int i = 0; i < 5; i++) {
          v += a * noise(p);
          p *= 2.0;
          a *= 0.5;
        }
        return v;
      }

      float lightning(vec2 uv, float t) {
        float y = uv.y;
        float n = fbm(vec2(uv.x * 3.0, t * 2.0));
        float ly = n * 0.5 + 0.5;
        float d = abs(y - ly);
        return smoothstep(0.05, 0.0, d);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        iTime = czm_frameNumber / 60.0 * speed;
        iResolution = czm_viewport.zw;

        vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        vec2 center = vec2(px.x, iResolution.y - px.y);
        vec2 p2 = (2.0 * center.xy - iResolution.xy) / min(iResolution.x, iResolution.y);
        uv = uv - p2;
        uv = uv * 2.0 * min(iResolution.x, iResolution.y) / pixel;

        float l = lightning(uv, iTime);
        float flash = step(0.95, hash(vec2(floor(iTime * 3.0), 0.0)));
        l *= flash;

        vec3 col = color.rgb * l * 3.0;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col));
        vec4 fragColor = vec4(col, alpha);

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
    const materialType = `VolumeAreaLightningMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#aabbff").withAlpha(0.9),
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
    throw new Error("VolumeAreaLightningEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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

export default VolumeAreaLightningEffect;
