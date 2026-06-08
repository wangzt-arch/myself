import * as Cesium from "cesium";

// 烟花特效 - 从地面发射到空中爆炸
// 使用 BoxGeometry 的 +X 面显示，发射阶段从下到上，爆炸阶段粒子扩散
class FireworkEffect {
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("FireworkEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height ?? 0,
      radius: options.radius ?? 300000,
      speed: options.speed ?? 0.8,
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

      // 2D rotation matrix
      mat2 rot(float a) {
        float c = cos(a), s = sin(a);
        return mat2(c, -s, s, c);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        czm_material material = czm_getDefaultMaterial(materialInput);
        float t = iTime * speed;

        // Cycle: launch (0-1.5s) -> explode (1.5-3s) -> fade (3-4s) -> repeat
        float cycle = 4.0;
        float ct = fract(t / cycle) * cycle;

        vec2 uv = materialInput.st;
        vec2 centered = uv - vec2(0.5, 0.5);

        vec3 finalColor = vec3(0.0);
        float finalAlpha = 0.0;

        // Launch phase: thin line shooting up from bottom
        if (ct < 1.2) {
          float launchProgress = ct / 1.2; // 0 -> 1
          float trailY = launchProgress * 0.85 + 0.05;
          float trailLen = 0.12;

          // Distance to trail line segment
          float distToTrail = abs(uv.x - 0.5);
          float inTrailY = step(trailY - trailLen, uv.y) * step(uv.y, trailY);

          // Trail fades at tail
          float trailFade = smoothstep(trailY - trailLen, trailY, uv.y);

          // Very thin center line, only visible head-on
          float trail = smoothstep(0.008, 0.0, distToTrail) * inTrailY * trailFade;

          // Trail color: bright white-yellow at the head, fading to orange
          vec3 trailColor = mix(vec3(1.0, 0.6, 0.1), vec3(1.0, 0.9, 0.7), trailFade);

          finalColor = trailColor * trail * 3.0;
          finalAlpha = trail;
        }

        // Explosion phase: particles burst from center
        if (ct >= 1.0 && ct < 3.5) {
          float explodeT = (ct - 1.0) / 2.5; // 0 -> 1 during explosion
          float explodeProgress = smoothstep(0.0, 0.3, explodeT); // quick burst
          float fadeOut = 1.0 - smoothstep(0.4, 1.0, explodeT);

          // Particle explosion: 60 particles bursting from center
          for (int i = 0; i < 60; i++) {
            float fi = float(i);
            float angle = hash(fi * 17.3) * 6.28318;
            float dist = 0.05 + hash(fi * 23.7) * 0.35;
            float particleSize = 0.018 + hash(fi * 31.1) * 0.022;
            float speedVar = 0.8 + hash(fi * 47.5) * 0.5;

            // Particle expansion from center
            float expand = explodeProgress * speedVar;
            float px = 0.5 + cos(angle) * dist * expand;
            float py = 0.65 + sin(angle) * dist * expand * 0.7;

            // Gravity effect: particles fall down over time
            py -= explodeT * explodeT * 0.15 * speedVar;

            // Distance to particle
            float d = length(uv - vec2(px, py));
            float particle = smoothstep(particleSize, particleSize * 0.2, d);

            // Particle color variation
            float hue = hash(fi * 59.3);
            vec3 pColor;
            if (hue < 0.2) pColor = vec3(1.0, 0.2, 0.1); // red
            else if (hue < 0.4) pColor = vec3(1.0, 0.6, 0.1); // orange
            else if (hue < 0.6) pColor = vec3(1.0, 0.9, 0.2); // yellow
            else if (hue < 0.8) pColor = vec3(0.2, 0.8, 1.0); // cyan
            else pColor = vec3(0.9, 0.2, 0.9); // magenta

            finalColor += pColor * particle * fadeOut * 2.5;
            finalAlpha += particle * fadeOut;
          }

          // Central flash at explosion start
          float flash = smoothstep(0.15, 0.0, explodeT) * smoothstep(0.0, 0.05, explodeT);
          float flashDist = length(centered - vec2(0.0, 0.15));
          float flashGlow = smoothstep(0.15, 0.0, flashDist) * flash;
          finalColor += vec3(1.0, 0.95, 0.8) * flashGlow * 4.0;
          finalAlpha += flashGlow;
        }

        finalAlpha = clamp(finalAlpha, 0.0, 1.0);

        material.diffuse = finalColor;
        material.alpha = finalAlpha * color.a;
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
    const materialType = `FireworkMaterial_${this._generateId()}`;
    const uniforms = {
      color: this.options.color || CesiumRef.Color.fromCssColorString("#ffaa44").withAlpha(0.95),
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
    // Position the box so its bottom aligns with the clicked height
    // Box local Z ranges from -0.5 to 0.5, scaled by realRadius * 3.0
    // So half-height = realRadius * 1.5, shift center up by that amount
    const halfHeight = this.options.radius * 1.5;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, height + halfHeight);

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

              // Show only on +X face
              float isTargetFace = step(0.9, v_normalMC.x);

              czm_materialInput materialInput;
              materialInput.normalEC = normalEC;
              materialInput.positionToEyeEC = positionToEyeEC;
              materialInput.st = v_st;

              czm_material material = czm_getMaterial(materialInput);

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

    // Get camera position and compute direction to face camera
    const camera = this.viewer.camera;
    const cameraPos = camera.positionWC;

    // Compute vector from position to camera (in world space)
    const toCamera = CesiumRef.Cartesian3.subtract(cameraPos, this.position, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(toCamera, toCamera);

    // Build a rotation matrix that makes +X face the camera
    // +X axis should point to camera
    const xAxis = toCamera;

    // Choose an up vector (use camera up or geodetic surface normal)
    const up = CesiumRef.Cartesian3.normalize(this.position, new CesiumRef.Cartesian3());

    // Compute yAxis = up × xAxis (perpendicular to both)
    const yAxis = CesiumRef.Cartesian3.cross(up, xAxis, new CesiumRef.Cartesian3());
    if (CesiumRef.Cartesian3.magnitude(yAxis) < 0.001) {
      // If xAxis is parallel to up, use a different up vector
      CesiumRef.Cartesian3.clone(CesiumRef.Cartesian3.UNIT_Z, yAxis);
    }
    CesiumRef.Cartesian3.normalize(yAxis, yAxis);

    // Compute zAxis = xAxis × yAxis
    const zAxis = CesiumRef.Cartesian3.cross(xAxis, yAxis, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(zAxis, zAxis);

    // Build rotation matrix from axes
    const rotationMatrix = CesiumRef.Matrix4.fromRotationTranslation(
      new CesiumRef.Matrix3(
        xAxis.x, yAxis.x, zAxis.x,
        xAxis.y, yAxis.y, zAxis.y,
        xAxis.z, yAxis.z, zAxis.z
      )
    );

    // Translation to position
    const translationMatrix = CesiumRef.Matrix4.fromTranslation(this.position);

    // Scale
    const scaleMatrix = CesiumRef.Matrix4.fromScale(
      new CesiumRef.Cartesian3(realRadius, realRadius, realRadius * 3.0)
    );

    // Combine: translation * rotation * scale
    let modelMatrix = CesiumRef.Matrix4.multiply(translationMatrix, rotationMatrix, new CesiumRef.Matrix4());
    modelMatrix = CesiumRef.Matrix4.multiply(modelMatrix, scaleMatrix, modelMatrix);
    this.primitive.modelMatrix = modelMatrix;
  }

  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("FireworkEffect: Cannot find Cesium.");
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

export default FireworkEffect;
