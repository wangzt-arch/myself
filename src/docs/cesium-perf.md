# Cesium WebGL / WebGPU 性能优化实战指南

CesiumJS 作为开源 3D 地理空间可视化引擎的标杆，其渲染管线深度依赖 WebGL（并正在向 WebGPU 迁移）。面对大规模地形、海量 3D Tiles、复杂矢量数据等场景，性能优化是保障流畅体验的核心课题。本文遵循"由浅入深、从通用到进阶"的原则，先介绍开箱即用的配置优化，再深入数据加载、图元渲染，最后探讨 WebGPU 的未来方向。

---

## 一、开箱即用的运行时调优

本节介绍的优化手段无需修改数据生产流程，仅需调整 Viewer 配置或场景参数即可生效，是性能调优的第一步。

### 1.1 帧率与分辨率动态调节

```js
// 1. 限制目标帧率（默认无限）
viewer.targetFrameRate = 30;

// 2. 降低渲染分辨率（以 0.5 倍分辨率渲染，再放大）
viewer.resolutionScale = 0.5;

// 3. 开启 requestRenderMode，仅在相机/数据变化时渲染
viewer.scene.requestRenderMode = true;
viewer.scene.maximumRenderTimeChange = 0.5; // 秒
```

`requestRenderMode` 是 Cesium 最具性价比的优化开关之一。静态场景下可将 GPU 占用从 100% 降至接近 0%。

### 1.2 场景级渲染开关

```js
// 关闭深度测试（仅当无地形遮挡需求时）
viewer.scene.globe.depthTestAgainstTerrain = false;

// 关闭大气层和雾效（若视觉方案允许）
viewer.scene.skyAtmosphere.show = false;
viewer.scene.fog.enabled = false;

// 关闭地面阴影
viewer.shadows = false;
```

### 1.3 相机移动时的动态降级

```js
// 相机移动时自动降低渲染质量，停止后恢复
viewer.scene.fog.enabled = true;
viewer.scene.fog.density = 0.0001;
viewer.scene.fog.screenSpaceErrorFactor = 2.0;
```

---

## 二、数据加载与流式优化

数据加载策略决定了首屏时间和内存峰值，是用户体验的关键路径。

### 2.1 3D Tiles 层级与 LOD 策略

3D Tiles 是 Cesium 的核心数据格式，其性能直接取决于瓦片组织的合理性：

- **合理设置 geometricError**：误差值决定相机距离多远时切换 LOD。过大导致细节丢失，过小导致过度加载
- **使用 Implicit Tiling（隐式瓦片）**：Cesium 1.9+ 支持的四叉树/八叉树隐式索引，减少 `tileset.json` 体积和解析时间
- **启用 Draco / KTX2 压缩**：
  - Draco：几何压缩，体积减少 50%~90%
  - KTX2 + Basis Universal：纹理压缩，显存占用减少 75%

```js
const tileset = await Cesium.Cesium3DTileset.fromUrl("tileset.json", {
  maximumScreenSpaceError: 16, // 默认 16，增大可减少加载瓦片数量
  dynamicScreenSpaceError: true, // 根据相机速度动态调整
  dynamicScreenSpaceErrorDensity: 0.00278,
  dynamicScreenSpaceErrorFactor: 4.0,
  skipLevelOfDetail: true, // 允许跳过中间 LOD，直接加载合适层级
  baseScreenSpaceError: 1024,
});
viewer.scene.primitives.add(tileset);
```

### 2.2 地形与影像优化

- **地形量化（Quantized-Mesh）**：使用 Cesium Ion 托管的地形服务已自动量化，自建地形时确保输出格式为 Quantized-Mesh 而非高度图
- **影像瓦片格式**：优先使用 WebP / JPEG（带 Mipmap），避免 PNG（体积大且无压缩优势）
- **影像图层数量控制**：每增加一层影像，就多一次全屏片元着色器采样。建议合并为单张贴图或使用 `ImageryLayer` 的 `show` 按需显隐

### 2.3 数据预加载与缓存

```js
// 预加载指定区域瓦片（适用于已知漫游路径的场景）
viewer.scene.globe.preloadSiblings = true;
viewer.scene.globe.preloadAncestors = true;

// 自定义缓存策略
Cesium.RequestScheduler.requestsByServer["tileserver.example.com:443"] = 18; // 提升并发数
```

---

## 三、实体图元与海量数据渲染优化

Cesium 的 `Entity` API 面向开发者友好，但每个 Entity 对应独立的 Primitive，海量数据下 Draw Call 爆炸。对于数万至数百万级别的点、线、面渲染，必须采用专门的优化策略。

### 3.1 Entity vs Primitive vs Custom Primitive

| API | 特点 | 适用规模 | 性能上限 |
|-----|------|---------|---------|
| **Entity** | 声明式、自动管理生命周期 | < 1,000 | 低，每个 Entity 独立 Draw Call |
| **Primitive** | 命令式、手动管理 Geometry/Appearance | < 10,000 | 中，可合批同类型几何 |
| **Custom Primitive** | 直接操作 WebGL/WebGPU 底层 | > 100,000 | 高，完全可控渲染管线 |
| **PointPrimitiveCollection** | 内置点云专用优化 | < 1,000,000 | 高，自动实例化 |
| **PolylineCollection** | 内置折线专用优化 | < 100,000 | 中，顶点合并绘制 |

> **核心原则**：Entity 适合交互式、动态更新的少量对象；海量静态数据应下沉到 Primitive 或 Custom Primitive 层。

### 3.2 点云与散点优化 — PointPrimitiveCollection

```js
const pointCollection = new Cesium.PointPrimitiveCollection();
viewer.scene.primitives.add(pointCollection);

// 批量添加 10 万+ 点
for (let i = 0; i < 100000; i++) {
  pointCollection.add({
    position: Cesium.Cartesian3.fromDegrees(lons[i], lats[i], alts[i]),
    color: Cesium.Color.fromCssColorString(colors[i]),
    pixelSize: sizes[i],
    outlineColor: Cesium.Color.BLACK,
    outlineWidth: 1,
  });
}
```

`PointPrimitiveCollection` 内部将所有点合并为单个 Draw Call，通过顶点属性传递位置和颜色，性能远超等量的 Entity。

**进阶优化**：
- **按颜色分组合批**：如果颜色种类有限，将同色点分批加入不同 `PointPrimitiveCollection`，减少 uniform 切换
- **LOD 策略**：远距离时增大 `pixelSize` 并减少点数量，或使用 Billboard 替代
- **GPU Picking**：使用 `Cesium.SceneTransforms` 将屏幕坐标反算到世界坐标，避免每帧遍历所有点做射线检测

### 3.3 折线与多边形优化

#### PolylineCollection（折线）

```js
const polylines = new Cesium.PolylineCollection();
viewer.scene.primitives.add(polylines);

// 批量添加，自动顶点合并
polylines.add({
  positions: Cesium.Cartesian3.fromDegreesArray([-75, 35, -125, 35]),
  width: 2,
  material: Cesium.Material.fromType('Color', { color: Cesium.Color.RED }),
});
```

#### GroundPrimitive（贴地多边形）

对于大面积行政区划、热力区域等贴地多边形，使用 `GroundPrimitive` 替代 `Entity.polygon`：

```js
const groundPrimitive = new Cesium.GroundPrimitive({
  geometryInstances: new Cesium.GeometryInstance({
    geometry: new Cesium.PolygonGeometry({
      polygonHierarchy: new Cesium.PolygonHierarchy(
        Cesium.Cartesian3.fromDegreesArray([...])
      ),
    }),
    attributes: {
      color: Cesium.ColorGeometryInstanceAttribute.fromColor(Cesium.Color.RED),
    },
  }),
  appearance: new Cesium.PerInstanceColorAppearance({
    closed: true,
    translucent: false,
  }),
});
viewer.scene.groundPrimitives.add(groundPrimitive);
```

`GroundPrimitive` 自动处理地形贴合，且支持 `PerInstanceColorAppearance` 实现单个 Primitive 内多实例颜色区分。

### 3.4 Billboard 与 Label 优化

Billboard（图标）和 Label（标签）是 GIS 中最常用的图元，也是性能瓶颈高发区：

```js
const billboards = new Cesium.BillboardCollection();
viewer.scene.primitives.add(billboards);

// 使用 Texture Atlas 合并贴图
billboards.add({
  position: position,
  image: 'icon.png', // 建议所有图标合并为一张 Atlas
  scale: 0.5,
  verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
});
```

**优化要点**：
- **合并贴图为 Texture Atlas**：Cesium 内部会自动将相同贴图的 Billboard 合批，但贴图切换会导致 Draw Call 拆分。建议使用工具（如 TexturePacker）将图标合并为单张 Atlas
- **距离缩放**：远距离时缩小或隐藏 Billboard
  ```js
  billboards.add({
    position: position,
    image: 'icon.png',
    scaleByDistance: new Cesium.NearFarScalar(1.5e2, 1.0, 1.5e7, 0.0), // 远距离淡出
  });
  ```
- **标签聚合（Clustering）**：Cesium 内置 `EntityCluster` 支持自动聚合
  ```js
  viewer.scene.postProcessStages.fxaa.enabled = true;
  dataSource.clustering.enabled = true;
  dataSource.clustering.pixelRange = 50;
  dataSource.clustering.minimumClusterSize = 3;
  ```

### 3.5 数据分页与视域加载

无论使用何种渲染技术，数据量超过视域可见范围时都应做分页：

```js
// 基于相机视域动态加载数据
viewer.camera.changed.addEventListener(() => {
  const rect = viewer.camera.computeViewRectangle();
  // 仅请求视域内的数据
  loadDataInRect(rect);
});

// 结合 Quadtree 空间索引
class DataQuadtree {
  constructor(level) {
    this.level = level;
    this.children = null;
    this.data = [];
  }
  insert(item, bounds) { /* 四叉树插入 */ }
  query(rect) { /* 视域查询 */ }
}
```

**关键指标**：
- 单帧渲染图元数控制在 5,000 以内（Entity）或 100,000 以内（Primitive/Custom）
- 视域外数据立即卸载，内存不累积
- 数据加载优先级：视域中心 > 边缘 > 视域外预加载

---

## 四、渲染管线深度优化

本节面向需要突破内置 Primitive 性能上限的场景，涉及更底层的渲染控制。

### 4.1 减少 Draw Call — 合批与实例化

WebGL 中每次 `drawArrays` / `drawElements` 都会触发 CPU-GPU 通信开销。Cesium 内部已通过以下机制自动优化，但开发者仍需在数据生产阶段配合：

| 技术 | 原理 | 适用场景 |
|------|------|----------|
| **Geometry Batching** | 将多个小几何体合并为单个 VBO 绘制 | 大量散点、图标、简单模型 |
| **Instancing** | 使用同一几何体 + 不同变换矩阵一次性绘制 | 树木、建筑、重复设施 |
| **Tipsify 算法** | 优化顶点缓存局部性，提升顶点着色器命中率 | 大规模地形、3D Tiles 模型 |

> Cesium 在处理 glTF / 3D Tiles 数据时，会自动应用 Tipsify 算法对顶点索引重排序，减少顶点缓存未命中。

### 4.2 着色器优化

Cesium 的 Material 和 Appearance 系统最终都会编译为 GLSL 着色器。优化要点：

- **避免在片元着色器中使用复杂分支**：GPU 对分支不友好，尽量使用 `mix()`、`step()` 等内置函数替代 `if/else`
- **减少纹理采样次数**：合并贴图（Texture Atlas）、使用 Mipmap、优先选择 `texture2DLOD` 的合适层级
- **精度修饰符显式声明**：对颜色、UV 使用 `lowp`，对位置、法线使用 `highp`，减少寄存器压力
- **Uniform Block 优化**：将高频更新的 uniform（如视图矩阵）与静态 uniform 分离，减少状态切换

### 4.3 自定义 Primitive — 突破性能天花板

当内置 Primitive 仍无法满足性能需求时，可直接操作 WebGL 底层：

```js
// 自定义 Primitive：直接绑定 VBO/IBO，绕过 Cesium 高层抽象
function CustomPointPrimitive(options) {
  this._positions = options.positions; // Float32Array [x,y,z, x,y,z, ...]
  this._colors = options.colors;       // Uint8Array [r,g,b,a, r,g,b,a, ...]
  this._count = options.count;
}

CustomPointPrimitive.prototype.update = function(frameState) {
  if (!this._va) {
    // 初始化 VertexArray
    const positionBuffer = Cesium.Buffer.createVertexBuffer({
      context: frameState.context,
      typedArray: this._positions,
      usage: Cesium.BufferUsage.STATIC_DRAW,
    });
    const colorBuffer = Cesium.Buffer.createVertexBuffer({
      context: frameState.context,
      typedArray: this._colors,
      usage: Cesium.BufferUsage.STATIC_DRAW,
    });
    this._va = new Cesium.VertexArray({
      context: frameState.context,
      attributes: [
        { index: 0, vertexBuffer: positionBuffer, componentsPerAttribute: 3 },
        { index: 1, vertexBuffer: colorBuffer, componentsPerAttribute: 4, normalize: true },
      ],
    });
    // 编译 Shader、创建 DrawCommand...
  }
  // 每帧将 DrawCommand 加入 frameState.commandList
  frameState.commandList.push(this._drawCommand);
};
```

**适用场景**：
- 百万级动态点实时更新（如航班、船舶轨迹）
- 自定义着色器效果（如热力图、风场粒子）
- GPU 驱动的数据筛选与交互

---

## 五、GPU 资源管理

### 5.1 纹理与显存

- **纹理尺寸为 2 的幂次**：确保 GPU 驱动可生成 Mipmap，提升采样效率
- **压缩纹理格式**：
  - WebGL 1: `DXT1/5`（桌面）、`ETC1`（移动端）
  - WebGL 2: `ASTC`（高质量、灵活块大小）
  - WebGPU: `BC7`、`ASTC` 均为核心支持
- **及时释放纹理**：移除 Entity / Primitive 时，调用 `viewer.scene.primitives.remove()` 并确保无循环引用

### 5.2 几何数据精简

- **顶点属性精简**：如不需要切线空间，移除 `TANGENT`；如不需要顶点色，移除 `COLOR_0`
- **索引类型选择**：顶点数 < 65536 时使用 `UNSIGNED_SHORT`（2字节），否则用 `UNSIGNED_INT`（4字节）
- **法线复用**：Cesium 支持 `OCT_ENCODED` 法线压缩，将 `FLOAT x3` 压缩为 `BYTE x2`

### 5.3 遮挡剔除与视锥剔除

Cesium 自动启用视锥剔除（Frustum Culling），但开发者可进一步利用：

- **背面剔除**：确保模型法线正确，Cesium 默认开启 `BACK` 面剔除
- **遮挡查询（Occlusion Query）**：对于大规模城市模型，可使用 `viewer.scene.globe.depthTestAgainstTerrain = true` 让地形遮挡地下物体
- **自定义包围盒**：对动态更新的 Primitive，手动设置准确的 `boundingVolume` 避免过度绘制

---

## 六、WebGPU 迁移与未来优化

### 6.1 WebGPU 带来的性能红利

Cesium 从 1.104 版本开始实验性支持 WebGPU 后端，其核心优势：

| 特性 | WebGL 限制 | WebGPU 优势 |
|------|-----------|-------------|
| **多线程命令录制** | 单线程 `gl.*` 调用 | `CommandEncoder` 可在 Worker 中预录制 |
| **计算着色器** | 不支持 | 可用于 GPU 驱动的剔除、LOD 计算、粒子系统 |
| **Bind Group** | 全局 uniform 状态机 | 显式资源绑定，减少状态切换 |
| **显式内存管理** | 隐式 GC 纹理/Buffer | `GPUBuffer`、`GPUTexture` 生命周期可控 |
| **Render Bundle** | 无 | 预录制命令序列，静态场景渲染开销趋近于零 |

### 6.2 Cesium 中启用 WebGPU

```js
const viewer = new Cesium.Viewer("cesiumContainer", {
  contextOptions: {
    requestWebgl1: false,
    webgl: {
      powerPreference: "high-performance",
    },
  },
});

// 实验性切换 WebGPU（Cesium 1.104+）
viewer.scene.context.webgl2 = false; // 关闭 WebGL2
// WebGPU 上下文需通过底层 API 显式创建，目前 Cesium 的 WebGPU 支持仍在演进中
```

> 截至 2025 年，Cesium 的 WebGPU 后端仍在积极开发中，生产环境建议继续使用 WebGL2 并关注官方版本更新。

### 6.3 WebGPU 专属优化方向

- **Compute Shader 预处理**：将地形高程解码、法线计算、瓦片可见性判断从 CPU 移至 Compute Shader
- **Indirect Draw**：通过 `drawIndirect` / `drawIndexedIndirect` 实现 GPU 驱动的合批，进一步降低 Draw Call
- **Timestamp Query**：利用 `GPUQuerySet` 精确测量 GPU 各阶段耗时，定位瓶颈

---

## 七、运行时监控与调试

### 7.1 内置性能面板

```js
// 显示 FPS
viewer.scene.debugShowFramesPerSecond = true;

// 打开 Inspector（含瓦片加载统计、Draw Call、GPU 内存等）
viewer.extend(Cesium.viewerCesiumInspectorMixin);
```

### 7.2 关键指标 checklist

| 指标 | 健康范围 | 优化方向 |
|------|---------|----------|
| FPS | > 30 | 降低 resolutionScale、减少瓦片加载 |
| Draw Calls | < 500 | 合批、实例化、减少 Entity 数量 |
| GPU Memory | < 80% 显存 | 压缩纹理、Draco 几何、及时释放 |
| Tile Requests | 平稳无突刺 | 调整 SSE、启用 skipLevelOfDetail |
| Frame Time (GPU) | < 16ms | 着色器简化、减少 overdraw |

---

## 八、总结

Cesium 的性能优化是一个系统工程，建议按照以下优先级逐步深入：

1. **先做运行时调优**：`requestRenderMode`、`resolutionScale`、`targetFrameRate` 等配置零成本见效
2. **再优化数据加载**：3D Tiles LOD、Draco/KTX2 压缩、影像格式选择，从源头减少数据量
3. **然后下沉图元层级**：Entity → Primitive → Custom Primitive，根据数据规模选择合适 API
4. **深入渲染管线**：合批、实例化、着色器优化，压榨每一帧的性能
5. **管好 GPU 资源**：纹理压缩、顶点精简、及时释放，避免内存泄漏
6. **关注 WebGPU 未来**：计算着色器、显式内存管理、Indirect Draw 将带来下一代性能突破

性能优化的最终目标是**在视觉质量与渲染开销之间找到最佳平衡点**，而非一味追求极致帧率。建议结合 Cesium Inspector 的实时数据，针对具体场景做针对性调优。
