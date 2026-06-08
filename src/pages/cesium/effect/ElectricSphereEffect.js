import * as Cesium from "cesium";

// 电磁弧
class ElectricSphereEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("ElectricSphereEffect: viewer is required");
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

      #define PI 3.1415926535
      #define PI2RAD 0.01745329252
      #define TWO_PI (2. * PI)

      float rands(float p) {
        return fract(sin(p) * 10000.0);
      }

      float noise(vec2 p) {
        float time = fract(czm_frameNumber * speed / 1000.0);
        float t = time / 20000.0;
        if (t > 1.0) t -= floor(t);
        return rands(p.x * 14. + p.y * sin(t) * 0.5);
      }

      vec2 sw(vec2 p) { return vec2(floor(p.x), floor(p.y)); }
      vec2 se(vec2 p) { return vec2(ceil(p.x), floor(p.y)); }
      vec2 nw(vec2 p) { return vec2(floor(p.x), ceil(p.y)); }
      vec2 ne(vec2 p) { return vec2(ceil(p.x), ceil(p.y)); }

      float smoothNoise(vec2 p) {
        vec2 inter = smoothstep(0.0, 1.0, fract(p));
        float s = mix(noise(sw(p)), noise(se(p)), inter.x);
        float n = mix(noise(nw(p)), noise(ne(p)), inter.x);
        return mix(s, n, inter.y);
      }

      float fbm(vec2 p) {
        float z = 2.0;
        float rz = 0.0;
        for (float i = 1.0; i < 6.0; i++) {
          rz += abs((smoothNoise(p) - 0.5) * 2.0) / z;
          z *= 2.0;
          p *= 2.0;
        }
        return rz;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        vec2 st = materialInput.st;
        vec2 st2 = materialInput.st;
        float time = fract(czm_frameNumber * speed / 1000.0);

        st *= 4.;
        float rz = fbm(st);
        st /= exp(mod(time * 2.0, PI));
        rz *= pow(15., 0.9);
        vec4 temp = vec4(0);
        temp = mix(color / rz, vec4(color.rgb, 0.1), 0.2);

        if (st2.s < 0.05) {
          temp = mix(vec4(color.rgb, 0.1), temp, st2.s / 0.05);
        }
        if (st2.s > 0.95) {
          temp = mix(temp, vec4(color.rgb, 0.1), (st2.s - 0.95) / 0.05);
        }

        material.diffuse = temp.rgb;
        material.alpha = temp.a * 2.0;
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
    const materialType = `ElectricSphereMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#00e5ff").withAlpha(0.9),
      speed: this.options.speed,
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
    throw new Error("ElectricSphereEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed || !this.material) return;
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

export default ElectricSphereEffect;
