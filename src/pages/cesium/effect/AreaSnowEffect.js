import * as Cesium from "cesium";

// 区域下雪 - 竖直面上的雪花飘落效果
class AreaSnowEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("AreaSnowEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 300000,
      speed: options.speed ?? 0.3,
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

      float hash(float n) {
        return fract(sin(n) * 43758.5453);
      }

      float hash2(vec2 p) {
        return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        float t = iTime * speed;

        vec2 uv = materialInput.st;

        // Generate multiple snowflakes
        float snow = 0.0;
        for (int i = 0; i < 80; i++) {
          float fi = float(i);

          // Random position seed
          float col = hash(fi * 13.7 + 7.3);
          float row = hash(fi * 29.3 + 11.1);
          float fallSpeed = 0.1 + hash(fi * 41.7) * 0.15;
          float driftSpeed = 0.3 + hash(fi * 53.3) * 0.5;
          float driftAmp = 0.02 + hash(fi * 67.1) * 0.04;
          float size = 0.01 + hash(fi * 79.3) * 0.016;

          // Snowflake position: falls down with horizontal drift (sine wave)
          float sx = col + sin(t * driftSpeed + fi * 2.0) * driftAmp;
          float sy = fract(row - t * fallSpeed);

          // Wrap x
          sx = fract(sx);

          // Distance to snowflake (circular dot)
          float dist = length(uv - vec2(sx, sy));
          float flake = smoothstep(size, size * 0.3, dist);

          // Fade at top and bottom of face
          float edgeFade = smoothstep(0.0, 0.08, uv.y) * smoothstep(1.0, 0.92, uv.y);

          snow += flake * edgeFade;
        }

        snow = clamp(snow, 0.0, 1.0);

        vec3 col = color.rgb * snow * 1.8;
        float alpha = snow * color.a;

        material.diffuse = col;
        material.alpha = alpha;
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
    const materialType = `AreaSnowMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ffffff").withAlpha(0.8),
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
    const { longitude, latitude, height } = this.options;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: CesiumRef.BoxGeometry.fromDimensions({
          vertexFormat: CesiumRef.VertexFormat.POSITION_NORMAL_AND_ST,
          dimensions: new CesiumRef.Cartesian3(1.0, 1.0, 1.0),
        }),
      }),
      appearance: new CesiumRef.MaterialAppearance({
        material: this.material,
        translucent: true,
        closed: true,
        vertexFormat: CesiumRef.VertexFormat.POSITION_NORMAL_AND_ST,
        vertexShaderSource: `
          in vec3 position3DHigh;
          in vec3 position3DLow;
          in vec3 normal;
          in vec2 st;
          in float batchId;

          out vec3 v_positionEC;
          out vec3 v_normalEC;
          out vec2 v_st;
          out vec3 v_normalMC;

          void main()
          {
              vec4 p = czm_computePosition();
              v_positionEC = (czm_modelViewRelativeToEye * p).xyz;
              v_normalEC = czm_normal * normal;
              v_normalMC = normal;
              v_st = st;
              gl_Position = czm_modelViewProjectionRelativeToEye * p;
          }
        `,
        fragmentShaderSource: `
          in vec3 v_positionEC;
          in vec3 v_normalEC;
          in vec2 v_st;
          in vec3 v_normalMC;

          void main()
          {
              vec3 positionToEyeEC = -v_positionEC;
              vec3 normalEC = normalize(v_normalEC);

              czm_materialInput materialInput;
              materialInput.normalEC = normalEC;
              materialInput.positionToEyeEC = positionToEyeEC;
              materialInput.st = v_st;

              czm_material material = czm_getMaterial(materialInput);

              out_FragColor = vec4(material.diffuse + material.emission, material.alpha);
          }
        `,
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
    const realRadius = this.options.radius;

    const translationMatrix = CesiumRef.Transforms.eastNorthUpToFixedFrame(this.position);
    const offset = new CesiumRef.Cartesian3(0.0, 0.0, realRadius * 1.5);
    const offsetMatrix = CesiumRef.Matrix4.fromTranslation(offset);
    const scaleMatrix = CesiumRef.Matrix4.fromScale(
      new CesiumRef.Cartesian3(realRadius, realRadius, realRadius * 3.0)
    );

    let modelMatrix = CesiumRef.Matrix4.multiply(translationMatrix, offsetMatrix, new CesiumRef.Matrix4());
    modelMatrix = CesiumRef.Matrix4.multiply(modelMatrix, scaleMatrix, modelMatrix);
    this.primitive.modelMatrix = modelMatrix;
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("AreaSnowEffect: Cannot find Cesium.");
  }

  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  startAnimation() {
    if (this.animationListener || !this.material) return;
    const startTime = Date.now();
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed || !this.material) return;
      const elapsed = (Date.now() - startTime) / 1000.0;
      this.material.uniforms.iTime = elapsed;
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

  show() {
    if (this.primitive) this.primitive.show = true;
  }

  hide() {
    if (this.primitive) this.primitive.show = false;
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

export default AreaSnowEffect;
