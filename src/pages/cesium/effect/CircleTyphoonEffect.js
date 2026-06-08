import * as Cesium from "cesium";
// 台风云团
class CircleTyphoonEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("CircleTyphoonEffect: viewer is required");
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

      mat2 r2d(float a) {
        float c = cos(a), s = sin(a);
        return mat2(c, s, -s, c);
      }

      float noise(vec2 uv) {
        return fract(sin(uv.x * 113. + uv.y * 412.) * 6339.);
      }

      vec3 noiseSmooth(vec2 uv) {
        vec2 index = floor(uv);
        vec2 pq = fract(uv);
        pq = smoothstep(0., 1., pq);

        float topLeft = noise(index);
        float topRight = noise(index + vec2(1, 0.));
        float top = mix(topLeft, topRight, pq.x);

        float bottomLeft = noise(index + vec2(0, 1));
        float bottomRight = noise(index + vec2(1, 1));
        float bottom = mix(bottomLeft, bottomRight, pq.x);

        return vec3(mix(top, bottom, pq.y));
      }

      vec3 genCloud(vec2 uv, vec2 orig, float bias) {
        vec3 col = noiseSmooth(uv * 4.);
        col += noiseSmooth(uv * 8.) * 0.5;
        col += noiseSmooth(uv * 16.) * 0.25;
        col += noiseSmooth(uv * 32.) * 0.125;
        col += noiseSmooth(uv * 64.) * 0.0625;
        col *= distance(orig, vec2(0.)) * bias;
        col *= smoothstep(0.2, 0.8, col);
        return col;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        vec2 uv = materialInput.st * 2.0 - 1.0;
        float time = iTime * speed;
        vec2 iResolution = czm_viewport.zw;
        uv *= 3.0;

        vec4 sky = vec4(vec3(1.0, 1.0, 1.0) * ((1.0 - uv.y) + 1.5) / 2.0, 1.0) / 2.1;
        vec2 orig = uv;

        vec2 uv2 = uv;
        vec2 uv3 = uv;

        uv *= r2d(time - distance(uv, vec2(0.)));
        uv2 *= r2d(time / 2.0 - distance(uv, vec2(0.)) * 1.2);
        uv3 *= r2d(time / 3.0 - distance(uv, vec2(0.)));

        uv2 += 0.4;
        uv3 += 0.3;

        vec4 col4 = vec4(genCloud(uv, orig, 2.65), 0.0);
        col4 = mix(col4, vec4(genCloud(uv2, orig, 3.8), 0.0), 0.5);
        col4 = mix(col4, vec4(genCloud(uv3, orig, 0.5), 0.0), 0.3);
        col4 = mix(1.0 - (col4 / 5.0), sky, 1.0 - col4);

        vec3 col = col4.rgb * color.rgb;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col.rgb));
        vec4 fragColor = vec4(col, alpha);

        material.diffuse = fragColor.rgb * 1.5;
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
    const materialType = `CircleTyphoonMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ff4d4d").withAlpha(0.9),
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
    throw new Error("CircleTyphoonEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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

export default CircleTyphoonEffect;
