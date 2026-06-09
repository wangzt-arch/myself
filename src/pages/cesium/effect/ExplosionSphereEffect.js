import * as Cesium from "cesium";

/**
 * ExplosionSphereEffect - Cesium 自定义 GLSL 爆炸特效
 *
 * 单面平面爆炸效果，随视角转动（billboard），隐藏立体/圆形部分。
 * 使用自定义 GLSL 绘制圆形爆炸冲击波效果。
 *
 * 适用场景：爆炸点、冲击波、能量爆发等三维地球可视化效果。
 *
 * 基础用法：
 * const effect = new ExplosionSphereEffect(viewer, {
 *   longitude: 116.395,
 *   latitude: 39.905,
 *   height: 100000,
 *   radius: 150000,
 *   speed: 0.7,
 *   Cesium,
 * })
 *
 * 销毁：
 * effect.destroy()
 *
 * @version 1.0.0
 * @requires Cesium
 */
class ExplosionSphereEffect {
  /**
   * 构造函数
   * @param {Cesium.Viewer} viewer - Cesium Viewer 实例。
   * @param {Object} options - 特效配置项。
   * @param {number} options.longitude - 爆炸中心经度，单位：度。
   * @param {number} options.latitude - 爆炸中心纬度，单位：度。
   * @param {number} [options.height] - 爆炸中心高度，单位：米；未传时使用 radius * heightOffset。
   * @param {number} [options.radius=120000] - 爆炸基准半径，单位：米。
   * @param {number} [options.speed=0.65] - 动画播放速度，数值越大爆炸循环越快。
   * @param {number} [options.heightOffset=0.55] - 未指定 height 时的高度偏移系数。
   * @param {Cesium.Color} [options.baseColor] - 爆炸热核心颜色，默认橙红色。
   * @param {Cesium.Color} [options.shockColor] - 爆炸冲击波颜色，默认亮黄色。
   * @param {Cesium.Color} [options.smokeColor] - 爆炸烟尘边缘颜色，默认暗褐色。
   * @param {boolean} [options.autoAnimate=true] - 是否创建后自动播放动画。
   * @param {Object} [options.Cesium] - Cesium 命名空间对象，ES Module 项目中建议显式传入。
   */
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error("ExplosionSphereEffect: viewer is required");
    }

    this.viewer = viewer;
    this._Cesium = options.Cesium || null;
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height,
      radius: options.radius ?? 120000,
      speed: options.speed ?? 0.65,
      heightOffset: options.heightOffset ?? 0.55,
      baseColor: options.baseColor,
      shockColor: options.shockColor,
      smokeColor: options.smokeColor,
      autoAnimate: options.autoAnimate !== false,
    };

    this.material = null;
    this.primitive = null;
    this.animationListener = null;
    this.startTime = null;
    this.isDestroyed = false;

    this._init();
  }

  /**
   * 返回 Cesium Material 使用的 GLSL shader 源码。
   * 单面圆形爆炸效果：中心爆发 -> 冲击波扩散 -> 消散
   * @private
   * @returns {string} GLSL 材质源码。
   */
  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform vec4 shockColor;
      uniform vec4 smokeColor;
      uniform float speed;
      uniform float time;

      float hash(float n) {
        return fract(cos(n) * 41415.92653);
      }

      float hash2(vec2 p) {
        return fract(cos(dot(p, vec2(127.1, 311.7))) * 43758.5453);
      }

      float noise(vec2 p) {
        vec2 i = floor(p);
        vec2 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        float a = hash2(i);
        float b = hash2(i + vec2(1.0, 0.0));
        float c = hash2(i + vec2(0.0, 1.0));
        float d = hash2(i + vec2(1.0, 1.0));
        return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        vec2 uv = materialInput.st;
        vec2 p = (uv - 0.5) * 2.0;
        float dist = length(p);

        // Circular mask - smooth edge
        float circularMask = smoothstep(1.0, 0.75, dist);

        // Animation cycle: burst -> expand -> fade
        float cycle = 2.0;
        float phase = fract(time * speed / cycle);

        // Phase timing:
        // 0.0 - 0.15: initial flash
        // 0.1 - 0.5: shockwave expansion
        // 0.4 - 0.8: debris/fade
        // 0.7 - 1.0: fade out

        float flash = smoothstep(0.0, 0.05, phase) * (1.0 - smoothstep(0.05, 0.2, phase));
        float shockwave = smoothstep(0.08, 0.15, phase) * (1.0 - smoothstep(0.4, 0.7, phase));
        float debris = smoothstep(0.3, 0.5, phase) * (1.0 - smoothstep(0.6, 0.9, phase));
        float fadeOut = 1.0 - smoothstep(0.65, 1.0, phase);

        // Central bright core
        float coreRadius = 0.08 + phase * 0.15;
        float core = smoothstep(coreRadius, 0.0, dist);

        // Noise turbulence
        float turb = noise(p * 4.0 + phase * 3.0) * 0.3
                   + noise(p * 8.0 - phase * 2.0) * 0.15;

        // Flying debris particles
        float debrisParticles = 0.0;
        for (int i = 0; i < 24; i++) {
          float fi = float(i);
          float angle = fi * 0.2618 + hash(fi * 17.3);
          float particleDist = (0.15 + hash(fi * 29.3) * 0.7) * phase * 1.6;
          float particleSize = 0.035 + hash(fi * 37.1) * 0.045;

          vec2 particlePos = vec2(cos(angle), sin(angle)) * particleDist;
          float d = length(p - particlePos);
          debrisParticles = max(debrisParticles, smoothstep(particleSize, 0.0, d));
        }

        // Combine all elements
        float intensity = core * 1.2 + debrisParticles * 0.5;
        intensity *= fadeOut;
        intensity += turb * 0.15 * shockwave;

        // Colors: white hot center -> yellow -> orange -> red -> dark edges
        vec3 whiteHot = vec3(1.0, 0.95, 0.8);
        vec3 brightYellow = vec3(1.0, 0.72, 0.15);
        vec3 orange = vec3(1.0, 0.5, 0.08);
        vec3 red = vec3(0.85, 0.15, 0.05);
        vec3 darkRed = vec3(0.35, 0.05, 0.02);

        // Color based on distance and phase
        float colorDist = dist * (1.0 - phase * 0.1);

        vec3 finalColor = whiteHot;
        finalColor = mix(finalColor, brightYellow, smoothstep(0.02, 0.1, colorDist));
        finalColor = mix(finalColor, orange, smoothstep(0.1, 0.25, colorDist));
        finalColor = mix(finalColor, red, smoothstep(0.25, 0.5, colorDist));
        finalColor = mix(finalColor, darkRed, smoothstep(0.5, 0.8, colorDist));

        // Core is brighter
        finalColor += whiteHot * core * 0.5;

        // Subtle brightness variation
        float brightness = noise(p * 5.0 + phase * 2.0) * 0.08 + 0.96;
        finalColor *= brightness;

        float alpha = intensity * circularMask;

        czm_material material = czm_getDefaultMaterial(materialInput);
        material.diffuse = finalColor;
        material.emission = finalColor * 1.8;
        material.alpha = clamp(alpha, 0.0, 0.9);
        material.specular = 0.0;
        return material;
      }
    `;
  }

  /**
   * 初始化材质、Primitive，并按配置启动动画。
   * @private
   */
  _init() {
    this._registerMaterial();
    this._createPrimitive();
    if (this.options.autoAnimate) {
      this.startAnimation();
    }
  }

  /**
   * 注册 Cesium 自定义材质，并创建当前实例专属 Material。
   * 每个实例使用独立 materialType，避免多个爆炸实例的 uniform 互相影响。
   * @private
   */
  _registerMaterial() {
    const CesiumRef = this._getCesium();
    const materialType = `ExplosionSphereMaterial_${this._generateId()}`;
    this._materialType = materialType;

    const uniforms = {
      color: this.options.baseColor || new CesiumRef.Color(1.0, 0.45, 0.04, 1.0),
      shockColor: this.options.shockColor || new CesiumRef.Color(1.0, 0.88, 0.25, 1.0),
      smokeColor: this.options.smokeColor || new CesiumRef.Color(0.18, 0.08, 0.03, 1.0),
      speed: this.options.speed,
      time: 0.0,
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

  /**
   * 创建单面平面 Primitive，随视角转动。
   * @private
   */
  _createPrimitive() {
    const CesiumRef = this._getCesium();
    const { longitude, latitude, height, radius, heightOffset } = this.options;
    const targetHeight = height !== undefined ? height : radius * heightOffset;
    this.position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, targetHeight);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: new CesiumRef.PlaneGeometry({
          vertexFormat: CesiumRef.MaterialAppearance.MaterialSupport.TEXTURED.vertexFormat,
        }),
      }),
      appearance: new CesiumRef.MaterialAppearance({
        material: this.material,
        translucent: true,
        faceForward: true,
        renderState: {
          depthTest: { enabled: true },
          depthMask: false,
          blending: CesiumRef.BlendingState.ALPHA_BLEND,
        },
      }),
      asynchronous: false,
    });

    this._syncPrimitive();
    this.viewer.scene.primitives.add(this.primitive);
  }

  /**
   * 同步 Primitive 的 modelMatrix，使平面始终面向相机。
   * @private
   */
  _syncPrimitive() {
    if (!this.primitive || !this.position) return;

    const CesiumRef = this._getCesium();
    const { radius } = this.options;
    const planeSize = radius * 2.0;

    // Get camera position and compute direction to face camera
    const camera = this.viewer.camera;
    const cameraPos = camera.positionWC;

    // Compute vector from position to camera
    const toCamera = CesiumRef.Cartesian3.subtract(cameraPos, this.position, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(toCamera, toCamera);

    // Build rotation matrix that makes plane face the camera
    // Plane normal (Z axis) should point to camera
    const zAxis = toCamera;

    // Choose up vector
    const up = CesiumRef.Cartesian3.normalize(this.position, new CesiumRef.Cartesian3());

    // Compute xAxis = up × zAxis
    const xAxis = CesiumRef.Cartesian3.cross(up, zAxis, new CesiumRef.Cartesian3());
    if (CesiumRef.Cartesian3.magnitude(xAxis) < 0.001) {
      CesiumRef.Cartesian3.clone(CesiumRef.Cartesian3.UNIT_X, xAxis);
    }
    CesiumRef.Cartesian3.normalize(xAxis, xAxis);

    // Compute yAxis = zAxis × xAxis
    const yAxis = CesiumRef.Cartesian3.cross(zAxis, xAxis, new CesiumRef.Cartesian3());
    CesiumRef.Cartesian3.normalize(yAxis, yAxis);

    // Build rotation matrix (plane faces camera)
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
    const scaleMatrix = CesiumRef.Matrix4.fromScale(new CesiumRef.Cartesian3(planeSize, planeSize, 1.0));

    // Combine: translation * rotation * scale
    let modelMatrix = CesiumRef.Matrix4.multiply(translationMatrix, rotationMatrix, new CesiumRef.Matrix4());
    modelMatrix = CesiumRef.Matrix4.multiply(modelMatrix, scaleMatrix, modelMatrix);
    this.primitive.modelMatrix = modelMatrix;
  }

  /**
   * 获取 Cesium 命名空间对象。
   * 优先使用模块导入的 Cesium，其次使用构造参数传入的 Cesium，最后尝试 window.Cesium。
   * @private
   * @returns {Object} Cesium 命名空间。
   */
  _getCesium() {
    if (Cesium && Cesium.Material) return Cesium;
    if (this._Cesium && this._Cesium.Material) return this._Cesium;
    if (typeof window !== "undefined" && window.Cesium) return window.Cesium;
    throw new Error("ExplosionSphereEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.");
  }

  /**
   * 生成材质类型后缀，确保每个实例拥有独立材质类型。
   * @private
   * @returns {string} 随机 ID。
   */
  _generateId() {
    return Math.random().toString(36).slice(2, 11);
  }

  /**
   * 开始动画。
   * 通过 viewer.scene.preRender 每帧更新材质 uniform 中的 time。
   */
  startAnimation() {
    if (this.animationListener || !this.material) return;

    this.startTime = Date.now();
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed) return;
      this.material.uniforms.time = (Date.now() - this.startTime) / 1000;
      this._syncPrimitive();
    });
  }

  /**
   * 停止动画并移除 preRender 监听。
   */
  stopAnimation() {
    if (this.animationListener) {
      this.viewer.scene.preRender.removeEventListener(this.animationListener);
      this.animationListener = null;
    }
  }

  /**
   * 设置动画播放速度。
   * @param {number} speed - 新的播放速度。
   */
  setSpeed(speed) {
    this.options.speed = speed;
    if (this.material) {
      this.material.uniforms.speed = speed;
    }
  }

  /**
   * 设置爆炸热核心颜色。
   * @param {Cesium.Color} color - Cesium 颜色对象。
   */
  setColor(color) {
    if (this.material) {
      this.material.uniforms.color = color;
    }
  }

  /**
   * 移动特效到新的经纬度位置。
   * 会重建 Primitive，但会复用已有材质和动画状态。
   * @param {number} longitude - 新经度，单位：度。
   * @param {number} latitude - 新纬度，单位：度。
   * @param {number} [height] - 新高度，单位：米；未传则沿用当前高度策略。
   */
  moveTo(longitude, latitude, height) {
    if (height !== undefined) {
      this.options.height = height;
    }

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive);
      this.primitive = null;
    }

    this.options.longitude = longitude;
    this.options.latitude = latitude;
    this._createPrimitive();
  }

  /**
   * 设置爆炸球体半径。
   * @param {number} radius - 新半径，单位：米。
   */
  setRadius(radius) {
    this.options.radius = radius;
    this.moveTo(this.options.longitude, this.options.latitude);
  }

  /**
   * 显示特效。
   */
  show() {
    if (this.primitive) {
      this.primitive.show = true;
    }
  }

  /**
   * 隐藏特效。
   */
  hide() {
    if (this.primitive) {
      this.primitive.show = false;
    }
  }

  /**
   * 销毁特效，释放 Primitive、材质和动画监听。
   * 外部删除特效或组件卸载时必须调用，避免 WebGL 资源残留。
   */
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

export default ExplosionSphereEffect;
