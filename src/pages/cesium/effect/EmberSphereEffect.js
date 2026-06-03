/**
 * EmberSphereEffect - Cesium 不规则流动余烬球体特效
 * 
 * 一个可复用的 CesiumJS 特效模块，创建带有噪声驱动不规则流动的深橙色余烬球体。
 * 适用于爆炸残留、魔法球体、能量核心等场景。
 * 
 * @version 1.0.0
 * @requires Cesium
 */
import * as Cesium from 'cesium'
class EmberSphereEffect {
  /**
   * 构造函数
   * @param {Cesium.Viewer} viewer - Cesium Viewer 实例
   * @param {Object} options - 配置选项
   * @param {number} options.longitude - 经度（度）
   * @param {number} options.latitude - 纬度（度）
   * @param {number} [options.height=0] - 高度（米），直接指定球心海拔高度
   * @param {number} [options.radius=250] - 球体半径（米）
   * @param {number} [options.speed=0.8] - 流动速度
   * @param {number} [options.heightOffset=0.3] - 高度偏移系数（0-1，球心抬升比例），当 height 未指定时生效
   * @param {Cesium.Color} [options.baseColor] - 基础颜色（默认深橙色）
   * @param {boolean} [options.autoAnimate=true] - 是否自动开启动画
   * @param {Object} [options.Cesium] - Cesium 命名空间对象（ES Module 方式使用时需要传入）
   */
  constructor(viewer, options = {}) {
    if (!viewer) {
      throw new Error('EmberSphereEffect: viewer is required')
    }

    this.viewer = viewer
    this._Cesium = options.Cesium || null
    this.options = {
      longitude: options.longitude ?? 116.395,
      latitude: options.latitude ?? 39.905,
      height: options.height,
      radius: options.radius ?? 250,
      speed: options.speed ?? 0.8,
      heightOffset: options.heightOffset ?? 0.3,
      baseColor: options.baseColor,
      autoAnimate: options.autoAnimate !== false,
    }

    this.material = null
    this.primitive = null
    this.animationListener = null
    this.startTime = null
    this.isDestroyed = false

    this._init()
  }

  /**
   * GLSL 着色器源码
   * @private
   */
  _getShaderSource() {
    return `
      uniform vec4 color;
      uniform float speed;
      uniform float time;

      // ---- Simplex 3D Noise ----
      vec3 mod289(vec3 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
      vec4 mod289(vec4 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
      vec4 permute(vec4 x) { return mod289(((x * 34.0) + 10.0) * x); }
      vec4 taylorInvSqrt(vec4 r) { return 1.79284291400159 - 0.85373472095314 * r; }

      float snoise(vec3 v) {
        const vec2 C = vec2(1.0 / 6.0, 1.0 / 3.0);
        const vec4 D = vec4(0.0, 0.5, 1.0, 2.0);
        vec3 i  = floor(v + dot(v, C.yyy));
        vec3 x0 = v - i + dot(i, C.xxx);
        vec3 g = step(x0.yzx, x0.xyz);
        vec3 l = 1.0 - g;
        vec3 i1 = min(g.xyz, l.zxy);
        vec3 i2 = max(g.xyz, l.zxy);
        vec3 x1 = x0 - i1 + C.xxx;
        vec3 x2 = x0 - i2 + C.yyy;
        vec3 x3 = x0 - D.yyy;
        i = mod289(i);
        vec4 p = permute(permute(permute(
                  i.z + vec4(0.0, i1.z, i2.z, 1.0))
                + i.y + vec4(0.0, i1.y, i2.y, 1.0))
                + i.x + vec4(0.0, i1.x, i2.x, 1.0));
        float n_ = 0.142857142857;
        vec3  ns = n_ * D.wyz - D.xzx;
        vec4 j = p - 49.0 * floor(p * ns.z * ns.z);
        vec4 x_ = floor(j * ns.z);
        vec4 y_ = floor(j - 7.0 * x_);
        vec4 x = x_ * ns.x + ns.yyyy;
        vec4 y = y_ * ns.x + ns.yyyy;
        vec4 h = 1.0 - abs(x) - abs(y);
        vec4 b0 = vec4(x.xy, y.xy);
        vec4 b1 = vec4(x.zw, y.zw);
        vec4 s0 = floor(b0) * 2.0 + 1.0;
        vec4 s1 = floor(b1) * 2.0 + 1.0;
        vec4 sh = -step(h, vec4(0.0));
        vec4 a0 = b0.xzyw + s0.xzyw * sh.xxyy;
        vec4 a1 = b1.xzyw + s1.xzyw * sh.zzww;
        vec3 p0 = vec3(a0.xy, h.x);
        vec3 p1 = vec3(a0.zw, h.y);
        vec3 p2 = vec3(a1.xy, h.z);
        vec3 p3 = vec3(a1.zw, h.w);
        vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2,p2), dot(p3,p3)));
        p0 *= norm.x; p1 *= norm.y; p2 *= norm.z; p3 *= norm.w;
        vec4 m = max(0.5 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
        m = m * m;
        return 105.0 * dot(m*m, vec4(dot(p0,x0), dot(p1,x1), dot(p2,x2), dot(p3,x3)));
      }

      czm_material czm_getMaterial(czm_materialInput materialInput) {
        vec2 uv = materialInput.st;
        float t = time * speed;

        // ---- 球面坐标映射 ----
        float theta = uv.x * 6.28318530718;
        float phi = uv.y * 3.14159265359;

        float sinPhi = sin(phi);
        float cosPhi = cos(phi);
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);
        vec3 spherePos = vec3(sinPhi * cosTheta, cosPhi, sinPhi * sinTheta);

        // ---- 全局时间流动 ----
        float flowTime = t * 0.3;

        // ---- 噪声驱动的表面变形 ----
        float distortion1 = snoise(spherePos * 2.0 + flowTime * 0.5) * 0.4;
        float distortion2 = snoise(spherePos * 5.0 + flowTime * 1.2) * 0.25;
        float distortion3 = snoise(spherePos * 10.0 + flowTime * 2.0) * 0.15;
        float totalDistortion = distortion1 + distortion2 + distortion3;

        float radialDist = 0.5 + totalDistortion;

        // ---- 流动的余烬区域 ----
        float flowNoise1 = snoise(spherePos * 3.0 + vec3(flowTime * 0.8, flowTime * 1.2, flowTime * 0.5));
        float flowNoise2 = snoise(spherePos * 6.0 + vec3(flowTime * 1.5, flowTime * 0.8, flowTime * 2.0));
        float flowNoise3 = snoise(spherePos * 12.0 + vec3(flowTime * 3.0, flowTime * 2.5, flowTime * 4.0));

        float flowIntensity = (flowNoise1 * 0.5 + flowNoise2 * 0.3 + flowNoise3 * 0.2) * 0.5 + 0.5;

        vec3 flowColor1 = vec3(0.9, 0.35, 0.02);
        vec3 flowColor2 = vec3(0.6, 0.18, 0.0);
        vec3 flowColor3 = vec3(0.4, 0.1, 0.0);

        vec3 flowCol = mix(flowColor2, flowColor1, flowNoise1 * 0.5 + 0.5);
        flowCol = mix(flowCol, flowColor3, flowNoise2 * 0.5 + 0.5);

        float flowShape = smoothstep(0.0, 0.6, flowIntensity) * smoothstep(1.2, 0.2, radialDist);

        vec3 expCol = flowCol;

        // 闪烁余烬
        float emberNoise = snoise(spherePos * 15.0 + flowTime * 5.0);
        float embers = smoothstep(0.6, 0.8, emberNoise) * smoothstep(0.8, 0.2, radialDist);
        vec3 emberColor = vec3(1.0, 0.45, 0.05);
        expCol = mix(expCol, emberColor, embers * 0.5);

        // 边缘暗化
        float edgeDarken = smoothstep(1.0, 0.3, radialDist);
        expCol *= 0.5 + edgeDarken * 0.5;

        // 透明度
        float alpha = flowShape * 0.8 + embers * 0.3;
        float edgeNoise = snoise(spherePos * 8.0 + flowTime * 3.0) * 0.3 + snoise(spherePos * 15.0 + flowTime * 5.0) * 0.2;
        alpha *= smoothstep(1.0 + edgeNoise, 0.5 + edgeNoise * 0.5, radialDist);
        alpha = clamp(alpha, 0.0, 1.0);

        czm_material material = czm_getDefaultMaterial(materialInput);
        material.diffuse = expCol * 2.5;
        material.emission = expCol * 4.0;
        material.alpha = alpha;
        material.specular = 0.0;
        return material;
      }
    `
  }

  /**
   * 初始化特效
   * @private
   */
  _init() {
    this._registerMaterial()
    this._createPrimitive()
    if (this.options.autoAnimate) {
      this.startAnimation()
    }
  }

  /**
   * 注册材质到 Cesium
   * @private
   */
  _registerMaterial() {
    const Cesium = this._getCesium()
    const materialType = `EmberSphereMaterial_${this._generateId()}`
    this._materialType = materialType

    // 使用 Cesium Material 内部 API 注册自定义材质
    if (Cesium.Material._materialCache) {
      Cesium.Material._materialCache.addMaterial(materialType, {
        fabric: {
          type: materialType,
          uniforms: {
            color: this.options.baseColor || new Cesium.Color(1.0, 0.5, 0.05, 1.0),
            speed: this.options.speed,
            time: 0.0,
          },
          source: this._getShaderSource(),
        },
        translucent: true,
      })
    }

    this.material = new Cesium.Material({
      fabric: {
        type: materialType,
        uniforms: {
          color: this.options.baseColor || new Cesium.Color(1.0, 0.5, 0.05, 1.0),
          speed: this.options.speed,
          time: 0.0,
        },
      },
      translucent: true,
    })
  }

  /**
   * 创建 Primitive
   * @private
   */
  _createPrimitive() {
    const Cesium = this._getCesium()
    const { longitude, latitude, height, radius, heightOffset } = this.options

    // 优先使用直接指定的高度，否则使用 radius * heightOffset
    const targetHeight = height !== undefined ? height : radius * heightOffset

    const sphereGeometry = new Cesium.SphereGeometry({
      radius: radius,
      vertexFormat: Cesium.MaterialAppearance.MaterialSupport.TEXTURED.vertexFormat,
    })

    // 计算世界坐标位置
    const position = Cesium.Cartesian3.fromDegrees(longitude, latitude, targetHeight)

    // 使用固定世界坐标变换，不随地球转动
    const modelMatrix = Cesium.Matrix4.fromTranslation(position)

    const geometryInstance = new Cesium.GeometryInstance({
      geometry: sphereGeometry,
      modelMatrix: modelMatrix,
    })

    const appearance = new Cesium.MaterialAppearance({
      material: this.material,
      translucent: true,
      faceForward: true,
      renderState: {
        depthTest: { enabled: true },
        depthMask: false,
        blending: Cesium.BlendingState.ALPHA_BLEND,
      },
    })

    this.primitive = new Cesium.Primitive({
      geometryInstances: geometryInstance,
      appearance: appearance,
      asynchronous: false,
    })

    this.viewer.scene.primitives.add(this.primitive)
  }

  /**
   * 获取 Cesium 全局对象
   * @private
   * @returns {Object} Cesium
   */
  _getCesium() {
    // 直接使用导入的 Cesium 模块（ES Module 方式）
    if (Cesium && Cesium.Material) {
      return Cesium
    }
    // 优先使用构造时传入的 Cesium 对象
    if (this._Cesium && this._Cesium.Material) {
      return this._Cesium
    }
    // 尝试从 viewer 获取 Cesium 构造函数
    const ViewerCesium = this.viewer.constructor
    if (ViewerCesium && ViewerCesium.Material) {
      return ViewerCesium
    }
    // 如果 viewer 是通过 import * as Cesium from 'cesium' 创建的
    // 需要全局 Cesium 对象
    if (typeof window !== 'undefined' && window.Cesium) {
      return window.Cesium
    }
    throw new Error('EmberSphereEffect: Cannot find Cesium. Pass Cesium via options.Cesium when using ES Modules.')
  }

  /**
   * 生成唯一 ID
   * @private
   * @returns {string}
   */
  _generateId() {
    return Math.random().toString(36).substr(2, 9)
  }

  /**
   * 开启动画
   */
  startAnimation() {
    if (this.animationListener || !this.material) return

    this.startTime = Date.now()
    this.animationListener = this.viewer.scene.preRender.addEventListener(() => {
      if (this.isDestroyed) return
      const elapsed = (Date.now() - this.startTime) / 1000.0
      this.material.uniforms.time = elapsed
    })
  }

  /**
   * 停止动画
   */
  stopAnimation() {
    if (this.animationListener) {
      this.viewer.scene.preRender.removeEventListener(this.animationListener)
      this.animationListener = null
    }
  }

  /**
   * 设置流动速度
   * @param {number} speed - 速度值
   */
  setSpeed(speed) {
    this.options.speed = speed
    if (this.material) {
      this.material.uniforms.speed = speed
    }
  }

  /**
   * 设置颜色
   * @param {Cesium.Color} color - Cesium 颜色对象
   */
  setColor(color) {
    if (this.material) {
      this.material.uniforms.color = color
    }
  }

  /**
   * 移动到新的位置
   * @param {number} longitude - 经度
   * @param {number} latitude - 纬度
   * @param {number} [height] - 高度（米），默认使用 radius * heightOffset
   */
  moveTo(longitude, latitude, height) {
    const Cesium = this._getCesium()
    const { radius, heightOffset } = this.options

    // 更新存储的高度值
    if (height !== undefined) {
      this.options.height = height
    }

    // 优先使用直接指定的高度，否则使用 radius * heightOffset
    const targetHeight = this.options.height !== undefined ? this.options.height : radius * heightOffset

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive)
    }

    this.options.longitude = longitude
    this.options.latitude = latitude

    const sphereGeometry = new Cesium.SphereGeometry({
      radius: radius,
      vertexFormat: Cesium.MaterialAppearance.MaterialSupport.TEXTURED.vertexFormat,
    })

    // 计算世界坐标位置，使用固定世界坐标变换
    const position = Cesium.Cartesian3.fromDegrees(longitude, latitude, targetHeight)
    const modelMatrix = Cesium.Matrix4.fromTranslation(position)

    const geometryInstance = new Cesium.GeometryInstance({
      geometry: sphereGeometry,
      modelMatrix: modelMatrix,
    })

    const appearance = new Cesium.MaterialAppearance({
      material: this.material,
      translucent: true,
      faceForward: true,
      renderState: {
        depthTest: { enabled: true },
        depthMask: false,
        blending: Cesium.BlendingState.ALPHA_BLEND,
      },
    })

    this.primitive = new Cesium.Primitive({
      geometryInstances: geometryInstance,
      appearance: appearance,
      asynchronous: false,
    })

    this.viewer.scene.primitives.add(this.primitive)
  }

  /**
   * 设置球体半径
   * @param {number} radius - 半径（米）
   */
  setRadius(radius) {
    this.options.radius = radius
    this.moveTo(this.options.longitude, this.options.latitude)
  }

  /**
   * 显示特效
   */
  show() {
    if (this.primitive) {
      this.primitive.show = true
    }
  }

  /**
   * 隐藏特效
   */
  hide() {
    if (this.primitive) {
      this.primitive.show = false
    }
  }

  /**
   * 销毁特效，释放资源
   */
  destroy() {
    this.isDestroyed = true
    this.stopAnimation()

    if (this.primitive) {
      this.viewer.scene.primitives.remove(this.primitive)
      this.primitive = null
    }

    if (this.material) {
      this.material.destroy()
      this.material = null
    }

    this.viewer = null
  }
}

export default EmberSphereEffect
