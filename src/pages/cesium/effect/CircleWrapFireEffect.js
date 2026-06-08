import * as Cesium from "cesium";

// 环绕火环
class CircleWrapFireEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleWrapFireEffect: viewer is required");
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

      #define TWO_PI 6.28318530718
      #define PI 3.14159265359

      float noise(vec2 p) {
        return fract(sin(p.x * 113. + p.y * 412.) * 6339.);
      }

      float fbm(vec2 p) {
        float a = 1.;
        float f = 1.;
        return a * noise(p)
          + a * 0.5 * noise(p * f * 2.)
          + a * 0.25 * noise(p * f * 4.)
          + a * 0.1 * noise(p * f * 8.);
      }

      float circle(vec2 p) {
        float r = length(p);
        float radius = 0.4;
        float height = 1.;
        float width = 150.;
        return height - pow(r - radius, 2.) * width;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        vec2 uv = materialInput.st * 2.0 - 1.0;
        float iTime = czm_frameNumber / 60.0 * speed;
        uv *= 0.5;

        vec2 st = vec2(
          atan(uv.y, uv.x),
          length(uv) * 1. + iTime * 0.1
        );

        st.x += st.y * 1.1;
        st.x = mod(st.x, TWO_PI);

        float n = fbm(st) * 1.5 - 1.;
        n = max(n, 0.1);
        float c = max(1. - circle(uv), 0.);
        float mask = smoothstep(0.48, 0.4, length(uv));

        c *= mask;
        c = clamp(c, -1.0, 3.0);
        vec3 col = color.rgb * c;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col.rgb));
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
    const materialType = `CircleWrapFireMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff4d00").withAlpha(0.9),
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
    throw new Error("CircleWrapFireEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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

export default CircleWrapFireEffect;
