import * as Cesium from "cesium";
//圆形预警
class CircleWarnEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleWarnEffect: viewer is required");
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

      float circle(float radius, vec2 center, vec2 uv) {
        float d = distance(center, uv);
        return 1.0 - smoothstep(radius - 1.0 / czm_viewport.zw.y, radius + 1.0 / czm_viewport.zw.y, d);
      }

      vec2 angleRadius(vec2 uv) {
        float anglePixel = atan(uv.y, uv.x);
        float lengthPixel = length(uv);
        return vec2(anglePixel, lengthPixel);
      }

      float filterPositive(float n) {
        return smoothstep(0.0, 0.005, n);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        vec2 uv = materialInput.st * 2.0 - 1.0;

        float radius = 0.3;
        float ringThick = 0.05;
        vec2 stPolar = angleRadius(uv);

        float sPolar = stPolar.x * 3.0 + iTime * speed * 10.0;
        float cosSPolar = filterPositive(cos(sPolar));
        vec3 sweepColor = vec3(cosSPolar);

        float inCircleAA = smoothstep(radius, radius + 0.005, stPolar.y);
        float smallCircleAA = smoothstep(radius - ringThick, radius - ringThick + 0.005, stPolar.y);
        vec3 outerCircle = 1.0 - vec3(inCircleAA);
        vec3 innerCircle = 1.0 - vec3(smallCircleAA);
        vec3 ringGap = outerCircle - innerCircle;

        vec3 colorMask = vec3(10.0, 1.5, 1.0);
        vec3 finalColor = sweepColor * ringGap;
        finalColor /= 10.0;
        finalColor *= colorMask;

        float centerCircleAA = smoothstep(0.1, 0.105, stPolar.y);
        vec3 centerCircleColor = 1.0 - vec3(centerCircleAA);
        centerCircleColor /= 10.0;
        centerCircleColor *= colorMask;

        float bubbleRadius = abs(sin(iTime * 3.0)) / 6.0;
        float bubbleCircleColor = circle(bubbleRadius, vec2(0.0), uv);
        vec3 bubbleColor = vec3(bubbleCircleColor) / 10.0 * colorMask;

        vec3 sourceColor = finalColor + centerCircleColor + bubbleColor;
        float intensity = max(max(sourceColor.r, sourceColor.g), sourceColor.b);
        vec3 warningColor = mix(sourceColor, sourceColor * color.rgb, 0.28);

        material.diffuse = warningColor;
        material.emission = warningColor;
        material.alpha = smoothstep(0.01, 0.12, intensity) * color.a;
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
    const materialType = `CircleWarnMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff2a1f"),
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
    throw new Error("CircleWarnEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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

export default CircleWarnEffect;
