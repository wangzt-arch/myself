import * as Cesium from "cesium";

/**
 * ExplosionSphereEffect - Cesium 自定义 GLSL 爆炸云团特效
 *
 * 一个可复用的 CesiumJS Primitive 特效模块，使用立体球形 Primitive 和
 * 自定义 GLSL 绘制从中心向各方向发散的白黄热核心、橙红火焰、烟尘暗边和碎星扰动。
 *
 * 适用场景：爆炸点、冲击波、能量爆发、告警热点等三维地球可视化效果。
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
   * @param {number} [options.radius=120000] - 爆炸云团基准半径，单位：米。
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
   * shader 内部通过噪声、时间参数和径向距离混合出热核心、冲击波、烟尘和火星。
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

      float gyroid(vec3 p) {
        return dot(cos(p), sin(p.yzx));
      }

      float fbm3(vec3 p) {
        float result = 0.0;
        float a = 0.5;
        for (int i = 0; i < 8; i++) {
          p.z += result * 0.1;
          result += abs(gyroid(p / a) * a);
          a /= 1.7;
        }
        return result;
      }

      vec3 palette(float v) {
        vec3 ember = vec3(0.95, 0.12, 0.02);
        vec3 orange = color.rgb * 1.85;
        vec3 yellow = shockColor.rgb * 2.35;
        vec3 whiteHot = vec3(1.0, 0.92, 0.48) * 3.1;
        vec3 fire = mix(ember, orange, smoothstep(0.18, 0.55, v));
        fire = mix(fire, yellow, smoothstep(0.48, 0.78, v));
        fire = mix(fire, whiteHot, smoothstep(0.78, 1.08, v));
        return fire;
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        vec2 uv = materialInput.st;
        vec2 p = (uv - 0.5) * 2.0;
        float dist = length(p);
        vec3 dir = normalize(vec3(p, 0.38 + smoothstep(1.2, 0.0, dist)));
        float phase = fract(time * speed);
        float growth = pow(phase, 0.22);
        float fade = 1.0 - pow(phase, 7.5);
        float burn = 1.0 - pow(phase, 0.45);

        vec3 flow = dir * (2.1 + growth * 3.2);
        flow += vec3(time * 0.34, -time * 0.22, time * 0.28);
        float shell = fbm3(flow);
        float detail = fbm3(dir * 8.0 + vec3(time * 0.9, -time * 0.55, time * 0.7));
        float plume = fbm3(dir * 3.4 + dir * growth * 2.6 + vec3(phase * 3.0));
        float edgeRipples = fbm3(vec3(p * 12.0, time * 0.8)) * 0.18
          + fbm3(vec3(p * 24.0, -time * 0.55)) * 0.08;

        float radialFlame = smoothstep(0.12, 1.18, shell + plume * 0.45 - burn * 0.45);
        float hotCore = smoothstep(1.15, 2.7, shell + detail * 0.35) * (1.0 - phase * 0.55);
        float smoke = smoothstep(0.18, 1.1, shell - burn * 0.82 + growth * 0.18);
        float shock = smoothstep(0.08, 0.0, abs(shell * 0.34 + plume * 0.15 - (0.22 + phase * 0.72))) * fade;
        float sparks = smoothstep(1.55, 2.25, detail + shell * 0.38) * smoothstep(0.08, 0.62, phase);

        vec3 fireColor = palette(shell * 0.24 + plume * 0.28 + hotCore * 0.5);
        vec3 smokeCol = smokeColor.rgb * (0.66 + plume * 0.22);
        vec3 finalCol = mix(fireColor, smokeCol, smoke * smoothstep(0.26, 1.0, phase));
        finalCol += shockColor.rgb * shock * 1.8;
        finalCol += vec3(1.0, 0.72, 0.2) * sparks * (1.0 - phase * 0.55);

        float alpha = radialFlame * fade * 0.62 + hotCore * 0.35 + smoke * 0.18 + shock * 0.26 + sparks * 0.18;
        alpha *= smoothstep(1.18 + edgeRipples, 0.34, dist);
        alpha *= 0.72 + smoothstep(0.16, 0.9, shell + detail * 0.25) * 0.28;
        alpha = clamp(alpha, 0.0, 0.92);

        czm_material material = czm_getDefaultMaterial(materialInput);
        material.diffuse = finalCol;
        material.emission = finalCol * (1.45 + hotCore * 1.35 + shock * 0.65);
        material.alpha = alpha;
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
   * 根据经纬度、高度和半径创建立体球形 Primitive。
   * Primitive 使用 MaterialAppearance 承载自定义 GLSL 材质。
   * @private
   */
  _createExplosionGeometry(CesiumRef, radius) {
    const segments = 112;
    const rings = [0, 0.24, 0.46, 0.68, 0.86, 1.0];
    const values = [];
    const sts = [];
    const indices = [];

    rings.forEach((ring, ringIndex) => {
      for (let index = 0; index < segments; index += 1) {
        const angle = (index / segments) * Math.PI * 2;
        const wave =
          1
          + Math.sin(angle * 3.0 + ringIndex * 0.8) * 0.18
          + Math.sin(angle * 7.0 - ringIndex * 0.45) * 0.1
          + Math.sin(angle * 13.0 + ringIndex * 1.7) * 0.055
          + Math.sin(angle * 19.0 - ringIndex * 2.1) * 0.035
          + Math.sin(angle * 31.0 + ringIndex * 0.35) * 0.022;
        const edgeNoise =
          ringIndex >= rings.length - 2
            ? 1 + Math.sin(angle * 23.0 + ringIndex) * 0.12 + Math.sin(angle * 41.0) * 0.055
            : 1;
        const edgeBoost = ringIndex === rings.length - 1 ? 1.18 : 1;
        const localRadius = radius * ring * wave * edgeBoost * edgeNoise;
        const height = radius * (
          Math.sin(ring * Math.PI) * 0.64
          + Math.max(0, 1 - ring) * 0.18
          + Math.sin(angle * 5.0 + ringIndex) * 0.08 * ring
          + Math.sin(angle * 17.0 - ringIndex * 0.6) * 0.045 * ring
        );

        values.push(Math.cos(angle) * localRadius, Math.sin(angle) * localRadius, height);
        sts.push(0.5 + Math.cos(angle) * ring * 0.5, 0.5 + Math.sin(angle) * ring * 0.5);
      }
    });

    for (let ringIndex = 0; ringIndex < rings.length - 1; ringIndex += 1) {
      const currentRing = ringIndex * segments;
      const nextRing = (ringIndex + 1) * segments;
      for (let index = 0; index < segments; index += 1) {
        const current = currentRing + index;
        const currentNext = currentRing + ((index + 1) % segments);
        const next = nextRing + index;
        const nextNext = nextRing + ((index + 1) % segments);
        indices.push(current, next, currentNext);
        indices.push(currentNext, next, nextNext);
      }
    }

    return new CesiumRef.Geometry({
      attributes: {
        position: new CesiumRef.GeometryAttribute({
          componentDatatype: CesiumRef.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: new Float64Array(values),
        }),
        st: new CesiumRef.GeometryAttribute({
          componentDatatype: CesiumRef.ComponentDatatype.FLOAT,
          componentsPerAttribute: 2,
          values: new Float32Array(sts),
        }),
      },
      indices,
      primitiveType: CesiumRef.PrimitiveType.TRIANGLES,
      boundingSphere: new CesiumRef.BoundingSphere(CesiumRef.Cartesian3.ZERO, radius * 1.45),
    });
  }

  _createPrimitive() {
    const CesiumRef = this._getCesium();
    const { longitude, latitude, height, radius, heightOffset } = this.options;
    const targetHeight = height !== undefined ? height : radius * heightOffset;
    const position = CesiumRef.Cartesian3.fromDegrees(longitude, latitude, targetHeight);
    const explosionGeometry = this._createExplosionGeometry(CesiumRef, radius);
    const modelMatrix = CesiumRef.Transforms.eastNorthUpToFixedFrame(position);

    this.primitive = new CesiumRef.Primitive({
      geometryInstances: new CesiumRef.GeometryInstance({
        geometry: explosionGeometry,
        modelMatrix,
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

    this.viewer.scene.primitives.add(this.primitive);
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
