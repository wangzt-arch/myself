import * as Cesium from "cesium";

// 区域闪电 - 3D立体四棱柱效果
// 闪电只在点击点上方的四棱柱范围内闪烁
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
      uniform float iTime;

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

      float lightningBolt(float x, float y, float t) {
        // More curved: use higher frequency noise with larger amplitude
        float n1 = fbm(vec2(t * 5.0, y * 6.0));
        float n2 = fbm(vec2(t * 3.0 + 100.0, y * 8.0 + 50.0));
        float lx = (n1 * 0.5 + n2 * 0.3) - 0.4;
        float d = abs(x - lx);
        // Thinner line
        float mainBolt = smoothstep(0.025, 0.0, d);
        float edgeFade = smoothstep(0.0, 0.1, y) * smoothstep(1.0, 0.9, y);
        return mainBolt * edgeFade;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        float t = iTime * speed;

        // Use st (2D texture coordinates) - range [0,1] on each face of BoxGeometry
        vec2 uv = materialInput.st;

        // Horizontal position centered at 0.5
        float x = uv.x - 0.5;
        // Vertical position
        float y = uv.y;

        // Lightning bolt
        float l = lightningBolt(x, y, t);

        // Flash effect - high frequency, almost always on
        float flash = step(0.5, hash(vec2(floor(t * 8.0), 0.0)));
        l *= flash;

        // Edge glow fade
        float edgeDist = max(abs(uv.x - 0.5), 0.0);
        float edgeFade = smoothstep(0.5, 0.3, edgeDist);
        l *= edgeFade;

        vec3 col = color.rgb * l * 5.0;
        col = (col + abs(col)) / 2.0;
        float alpha = smoothstep(0.0, 1.0, length(col));
        vec4 fragColor = vec4(col, alpha);

        material.diffuse = fragColor.rgb * 2.5;
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

    // BoxGeometry generates 'position', 'normal', 'st' attributes.
    // MaterialAppearance's default vertex shader expects 'position3DHigh'/'position3DLow'
    // which BoxGeometry does NOT provide. So we MUST provide custom vertex/fragment shaders
    // that use 'position' instead, and set vertexFormat to POSITION_NORMAL_AND_ST.
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

              // Only show lightning on one vertical face (+X face, normal = (1,0,0))
              float isTargetFace = step(0.9, v_normalMC.x);

              czm_materialInput materialInput;
              materialInput.normalEC = normalEC;
              materialInput.positionToEyeEC = positionToEyeEC;
              materialInput.st = v_st;

              czm_material material = czm_getMaterial(materialInput);

              // Only render lightning on target face, other faces are transparent
              float alpha = material.alpha * isTargetFace;
              out_FragColor = vec4(material.diffuse + material.emission, alpha);
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
    // Tall box: width=radius, depth=radius, height=3*radius
    const translationMatrix = CesiumRef.Transforms.eastNorthUpToFixedFrame(this.position);
    // Offset by half height so the bottom of the box is at the click point
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
    throw new Error("VolumeAreaLightningEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
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
