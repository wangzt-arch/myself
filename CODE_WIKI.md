# Code Wiki - myself 个人技术实验室

## 项目概述

**项目名称**: myself
**版本**: 0.1.0
**类型**: React 单页应用 (SPA)
**描述**: 个人技术作品集与实验室，展示前端工程实践、3D可视化、流程图编辑、技术文档和媒体案例。
**首页**: https://wangzt-arch.github.io/myself

---

## 技术栈

### 核心框架
- **React**: ^18.0.0 - UI框架
- **React Router**: ^6.2.2 - 路由管理
- **Webpack**: ^5.64.4 - 构建工具

### 3D 可视化
- **Three.js**: ^0.163.0 - 3D渲染引擎
- **@react-three/fiber**: ^8.16.1 - React Three.js 绑定
- **@react-three/drei**: ^9.105.3 - React Three.js 工具集
- **Cesium**: ^1.141.0 - 3D地球与GIS可视化

### 图表与数据可视化
- **ECharts**: ^5.3.3 - 数据可视化图表库

### 流程图
- **@logicflow/core**: ^1.2.27 - 流程图编辑器核心
- **@logicflow/extension**: ^1.2.27 - 流程图扩展组件

### 文档与渲染
- **react-markdown**: ^8.0.1 - Markdown渲染
- **react-syntax-highlighter**: ^15.5.0 - 代码高亮
- **remark-gfm**: ^3.0.1 - GitHub风格Markdown

### 其他重要依赖
- **axios**: ^1.2.1 - HTTP请求
- **party-js**: ^2.1.2 - 粒子效果
- **tailwindcss**: ^3.0.2 - CSS框架
- **sass/sass-loader**: 样式预处理
- **pdfjs-dist**: PDF渲染

---

## 项目架构

```
myself/
├── public/                 # 静态资源
│   ├── china.json         # 中国行政区划GeoJSON
│   ├── index.html         # HTML模板
│   └── manifest.json      # PWA配置
├── src/
│   ├── api/               # API接口
│   │   └── index.js       # 翻译API调用
│   ├── components/        # 通用组件
│   │   ├── Header/        # 导航栏
│   │   ├── Line/          # 线条组件
│   │   ├── Loading*/      # 加载动画组件
│   │   ├── OpenScreen/    # 启动画面
│   │   ├── cube-box/     # 立方体展示
│   │   ├── lanterns/     # 灯笼特效
│   │   └── wzt-button/    # 自定义按钮
│   ├── docs/              # 技术文档MD文件
│   │   ├── index.js       # 文档索引
│   │   ├── *.md           # 各分类文档
│   │   └── ai-*.md        # AI相关文档
│   ├── hooks/             # 自定义Hooks
│   │   └── useScrollReveal.js  # 滚动入场动画
│   ├── images/            # 图片资源
│   ├── pages/             # 页面组件
│   │   ├── home/          # 首页
│   │   ├── docs/          # 文档中心
│   │   ├── about/         # 关于/简历
│   │   ├── cesium/        # Cesium 3D地球
│   │   ├── model-preview/ # 3D模型预览
│   │   ├── logicflow/     # 流程图编辑
│   │   ├── video/         # 视频案例
│   │   ├── translate/     # 翻译工具
│   │   ├── yq-distribution/  # 数据可视化
│   │   └── no-found/      # 404页面
│   ├── utils/             # 工具函数
│   │   ├── index.js       # 通用工具
│   │   └── vis3d.js       # 3D可视化工具
│   ├── videos/            # 视频素材
│   ├── App.js             # 根组件/路由配置
│   ├── index.js           # 入口文件
│   └── index.css          # 全局样式
├── scripts/                # 构建脚本
│   ├── start.js           # 开发服务器
│   ├── build.js           # 生产构建
│   └── test.js            # 测试
├── config/                # Webpack配置
│   ├── webpack.config.js
│   ├── webpackDevServer.config.js
│   └── jest/              # Jest配置
└── package.json
```

---

## 路由结构

| 路径 | 组件 | 功能描述 |
|------|------|----------|
| `/` | Home | 首页导航 |
| `/home` | Home | 首页导航 |
| `/docs` | Docs | 技术文档中心 |
| `/about` | About | 个人简历PDF |
| `/chart` | Chart (YqDistribution) | ECharts数据可视化 |
| `/translate` | Translate | 彩云翻译工具 |
| `/preview` | ModelPreview | 3D模型预览/智慧园区 |
| `/logicflow` | LogicFlow | 流程图编辑器 |
| `/video` | Video | 视频案例展示 |
| `/cesium` | CesiumPage | Cesium 3D地球 |
| `*` | NoFound | 404页面 |

---

## 主要模块详解

### 1. 首页模块 (src/pages/home)

**文件**: `src/pages/home/index.jsx`

**功能**: 作为作品导航首页，展示项目入口和能力概览。

**核心组件**:
```jsx
function Home()
```

**关键数据**:
- `featureItems`: 6个功能入口卡片配置
- `skillItems`: 技术栈列表

**依赖组件**:
- `Header`: 通用导航栏
- `useScrollReveal`: 滚动入场动画Hook

---

### 2. 文档中心 (src/pages/docs)

**文件**: `src/pages/docs/index.jsx`

**功能**: Markdown文档阅读器，支持搜索、目录导航、代码高亮复制。

**核心函数**:

| 函数名 | 功能 |
|--------|------|
| `slugify(text)` | 生成URL友好的slug |
| `getPlainText(children)` | 提取React children纯文本 |
| `getHeadings(markdown)` | 解析Markdown标题生成目录 |
| `getReadingMinutes(markdown)` | 计算阅读时间 |

**特性**:
- 分类展示文档列表
- 关键词搜索（标题/分类/标签/正文）
- 侧边目录导航
- 代码块一键复制

**依赖**:
- `react-markdown`: Markdown渲染
- `react-syntax-highlighter`: 代码高亮
- `remark-gfm`: GFM语法支持

---

### 3. Cesium 3D地球 (src/pages/cesium)

**文件**: `src/pages/cesium/index.jsx`

**功能**: 基于CesiumJS的3D地球可视化，支持城市标记、卫星轨迹、无人机巡检、WebGL特效。

**核心配置**:
```javascript
CESIUM_TOKEN     // Cesium Ion访问令牌
INITIAL_CAMERA  // 初始相机位置
VIEWER_OPTIONS  // Cesium Viewer配置
STATS           // 信息面板统计数据
```

**核心数据结构**:
```javascript
CITIES_DATA     // 城市点位数据
INITIAL_LAYERS   // 初始图层可见性
DRONE_ROUTE_OPTIONS  // 无人机航线选项
```

**主要图层**:
- `cityMarkers`: 城市标记
- `provinceLabels`: 省份标签
- `satellite`: 卫星轨迹
- `flightLines`: 城市飞线
- `coverage`: 覆盖范围

**核心Helper函数**:

| 函数名 | 功能 |
|--------|------|
| `createMouseMoveHandler` | 创建鼠标移动事件处理器 |
| `createCityClickHandler` | 创建城市点击事件处理器 |
| `getMapClickPosition` | 获取地图点击的地球坐标 |
| `getPickedCityName` | 获取点击实体的城市名 |
| `getPopupPosition` | 计算弹窗位置（防溢出） |
| `loadIonImagery` | 加载卫星影像 |
| `loadChinaBoundary` | 加载中国行政区划GeoJSON |
| `createProvinceLabels` | 创建省份名称标签 |
| `flyToInitialCamera` | 飞回初始视角 |
| `createCityMarkers` | 创建城市标记 |
| `applyInitialLayerVisibility` | 应用初始图层可见性 |
| `getSatelliteTarget` | 获取卫星实体 |
| `getSatelliteCameraDestination` | 计算卫星跟随相机位置 |
| `updateSatelliteFollowCamera` | 更新卫星跟随相机 |
| `addSun` | 添加太阳特效 |

---

### 4. Cesium特效系统 (src/pages/cesium/effects.js)

**文件**: `src/pages/cesium/effects.js`

**功能**: 管理Cesium地图上的WebGL特效。

**特效类型** (`EFFECT_TYPES`):
- `flame`: 火球
- `fireball`: 火焰
- `explosion`: 爆炸
- `volumeSmoke`: 体积烟
- `circleBlast`: 圆形爆炸
- `circleWarn`: 圆形预警
- `circleDisturb`: 干扰圈
- `circleDiffusion`: 圆形扩散
- `electricSphere`: 电磁弧
- `areaRain`: 区域下雨
- `areaSnow`: 区域下雪
- `firework`: 烟花
- `energyWall`: 能量墙
- `alarmWall`: 警戒墙
- `glow`: 彩虹墙
- 等30+种特效

**核心API**:
```javascript
// 创建特效
createWebGLEffect(viewer, type, position, index)
// 移除特效
removeWebGLEffect(viewer, effect)
```

---

### 5. Cesium特效类 (src/pages/cesium/effect/)

**目录**: `src/pages/cesium/effect/`

包含30+个特效类文件，继承自 `RotatingCircleEffectBase`:

| 类名 | 特效类型 |
|------|----------|
| `EmberSphereEffect` | 燃烧粒子球体 |
| `BallOfFireEffect` | 火球 |
| `ExplosionSphereEffect` | 爆炸球体 |
| `VolumeSmokeEffect` | 体积烟 |
| `CircleBlastEffect` | 圆形爆炸 |
| `CircleWarnEffect` | 圆形预警 |
| `CircleDiffuseFireEffect` | 圆形散射火 |
| `CircleDisturbEffect` | 干扰圈 |
| `CircleElectricAreaEffect` | 电域 |
| `CircleDiffusionEffect` | 圆形扩散 |
| `CircleGifEffect` | GIF静态爆炸 |
| `CircleHelicalLineEffect` | 螺旋线 |
| `CircleFlashEffect` | 圆形闪烁 |
| `CircleRotateColorLineEffect` | 旋转彩线 |
| `CircleRotateGarlandEffect` | 旋转花环 |
| `CircleRotateHaloEffect` | 旋转光圈 |
| `CircleTyphoonEffect` | 台风云团 |
| `CircleWrapFireEffect` | 环绕火环 |
| `ElectricSphereEffect` | 电磁弧球 |
| `VolumeAreaLightningEffect` | 区域闪电 |
| `VolumeArrowAttackEffect` | 万箭穿心 |
| `VolumeDiffuseFireEffect` | 体散射火 |
| `VolumeDisturbLineEffect` | 干扰线域 |
| `AreaRainEffect` | 区域雨 |
| `AreaSnowEffect` | 区域雪 |
| `FireworkEffect` | 烟花 |

---

### 6. 3D模型预览 (src/pages/model-preview)

**文件**: `src/pages/model-preview/ModelPreviewTab.jsx`

**功能**: 基于React Three Fiber的GLB模型查看器，支持材质调节、爆炸图。

**核心组件**:
```javascript
Model          // GLTF模型渲染组件
ModelPreviewTab // 主组件
Loader         // 加载进度组件
```

**共享参数系统** (`sharedParamsRef`):
```javascript
{
  modelScale,     // 模型缩放
  explodeFactor,  // 爆炸图系数
  wireframe,      // 线框模式
  modelOpacity,   // 透明度
  modelColor,     // 模型颜色
  metalness,      // 金属度
  roughness       // 粗糙度
}
```

**支持模型** (src/pages/model-preview/models/):
- `xiaomi_su7_max.glb` - 小米SU7 (支持爆炸图)
- `无人机.glb`
- `步枪.glb`、`狙击枪_ar-15.glb` 等武器模型
- `高射炮.glb`、`火箭筒.glb` 等军事模型
- `防弹衣.glb`
- `supermarket.glb`

**爆炸图逻辑**:
```javascript
getExplodeDirection(name)  // 根据节点名称判断爆炸方向
// 规则:
// - wheel/hub/brake: 水平爆炸
// - frontkit: 向前
// - rearkit: 向后
// - hood: 向上前方
// - interior: 向上
```

---

### 7. 智慧园区 (src/pages/model-preview/smart-park)

**文件**: `src/pages/model-preview/smart-park/index.jsx`

**功能**: 数字孪生智慧园区3D场景，包含建筑、数据面板、环境监测。

**核心组件**:
```javascript
SmartParkTab      // 主Tab组件
BuildingDetailPopup  // 建筑详情弹窗
Scene             // 3D场景 (Scene.jsx)
DataPanel         // 数据面板 (DataPanel.jsx)
Toolbar           // 工具栏 (Toolbar.jsx)
Ground            // 地面 (Ground.jsx)
Building          // 建筑 (Building.jsx)
```

**功能特性**:
- 视角模式切换
- 天气效果模拟
- 时间模拟
- 建筑点击查看详情
- 环境监测显示

---

### 8. 流程图编辑器 (src/pages/logicflow)

**文件**: `src/pages/logicflow/index.jsx`

**功能**: 基于LogicFlow的流程图编辑工具，支持自定义节点样式、撤销重做、导出。

**使用的LogicFlow扩展**:
- `Menu`: 右键菜单
- `DndPanel`: 拖拽面板
- `SelectionSelect`: 框选
- `Snapshot`: 快照/导出

**自定义节点类型**:
- `TaskZzrw`: 任务节点-任务
- `TaskZzxd`: 任务节点-站点
- `TaskZzhd`: 任务节点-活动
- `TaskZzjd`: 任务节点-节点

**工具按钮**:
- 重做 (redo)
- 撤销 (undo)
- 放大 (zoomIn)
- 缩小 (zoomOut)
- 自适应 (fitView)
- 框选 (selection)

**支持功能**:
- 拖拽添加节点
- 右键删除
- XML/PNG导出
- 自定义节点样式

---

### 9. 数据可视化 (src/pages/yq-distribution)

**文件**: `src/pages/yq-distribution/index.jsx`

**功能**: 基于ECharts的数据可视化驾驶舱，包含地图、折线图、柱状图、饼图等。

**图表类型**:
| 图表 | 函数 | 说明 |
|------|------|------|
| 地图 | `buildMapOption` | 中国地图散点 |
| 折线图 | `buildLineOption` | 访问趋势 |
| 柱状图 | `buildBarOption` | 渠道贡献 |
| 饼图 | `buildPieOption` | 终端占比 |
| 雷达图 | `buildRadarOption` | 能力雷达 |
| 仪表盘 | `buildGaugeOption` | 转化效率 |
| 散点图 | `buildScatterOption` | 分布散点 |

**数据**:
- `cityScatterData`: 城市活跃度
- `monthlyTrend`: 月度趋势
- `channelData`: 渠道数据
- `deviceData`: 终端数据
- `metrics`: 关键指标

---

### 10. 翻译工具 (src/pages/translate)

**文件**: `src/pages/translate/index.jsx`

**功能**: 基于彩云翻译API的文本翻译工具。

**API调用**:
```javascript
getTranslate(text, language)
// language: 'en' (中->英) 或 'zh' (英->中)
```

**功能特性**:
- 实时翻译（600ms防抖）
- 语言切换
- 翻译历史（localStorage存储）
- 复制功能
- Enter键触发翻译

---

### 11. 视频案例 (src/pages/video)

**文件**: `src/pages/video/index.jsx`

**功能**: 视频素材瀑布流展示页面。

**核心组件**:
```javascript
VideoPage    // 主页面
VideoPlayer  // 播放器组件
```

**特性**:
- 动态import视频文件
- 分类筛选
- 瀑布流布局
- hover播放

---

### 12. 关于页面 (src/pages/about)

**文件**: `src/pages/about/index.jsx`

**功能**: PDF简历预览器。

**依赖**:
- `@mikecousins/react-pdf`: PDF渲染
- `react-pdf-js`: PDF.js封装

**功能**:
- 分页浏览
- 缩放控制
- 响应式布局

---

### 13. 通用Header组件 (src/components/Header)

**文件**: `src/components/Header/index.jsx`

**功能**: 全局导航栏组件。

**导航项**:
```javascript
[
  { text: "首页", path: "/home" },
  { text: "关于", path: "/about" },
  { text: "文档", path: "/docs" },
  { text: "图表", path: "/chart" },
  { text: "翻译", path: "/translate" },
  { text: "三维场景", path: "/preview" },
  { text: "流程图", path: "/logicflow" },
  { text: "视频案例", path: "/video" },
  { text: "3D地球", path: "/cesium" }
]
```

**特性**:
- 当前路由高亮
- 移动端汉堡菜单
- GitHub链接

---

### 14. 滚动入场动画Hook (src/hooks/useScrollReveal)

**文件**: `src/hooks/useScrollReveal.js`

**功能**: 监听元素进入视口时触发入场动画。

**API**:
```javascript
const { ref, isVisible } = useScrollReveal({
  threshold: 0.15,      // 触发阈值
  rootMargin: "0px 0px -40px 0px"  // 视口边距
})
```

---

### 15. 文档系统 (src/docs)

**文件**: `src/docs/index.js`

**功能**: 管理所有技术文档的元数据和内容。

**文档分类**:
| 分类 | 文档数 |
|------|--------|
| AI 工具 | 4 |
| 前端基础 | 3 |
| 工程效率 | 2 |
| 移动端 | 1 |
| 能力地图 | 1 |
| GIS 可视化 | 3 |
| 面试相关 | 7 |

**文档列表** (共21篇):
- AI与Codex入门
- Codex下载注册指南
- AI提示语最佳实践
- AI圈子热词百科
- 常用正则
- ES6+新特性概览
- 函数式编程
- 常用JS工具
- 鸿蒙OS
- 常用指令
- 掌握技能
- OpenLayers方法
- Cesium方法
- Cesium WebGL性能优化
- JS/HTML/CSS/Vue/React/TypeScript面试题
- 前端工程化面试题

---

### 16. API层 (src/api)

**文件**: `src/api/index.js`

**功能**: 封装外部API调用。

**当前API**:
```javascript
getTranslate(text, language)
// 调用彩云翻译API
// 端点: https://api.interpreter.caiyunai.com/v1/translator
```

---

### 17. 工具函数 (src/utils)

**文件**: `src/utils/index.js`

**功能**: 通用工具函数。

```javascript
isPC()  // 检测是否为PC端
```

**文件**: `src/utils/vis3d.js`

**功能**: 3D可视化相关工具函数（Cesium坐标转换等）。

**主要方法**:
```javascript
cartesianToLnglat(cartesian, viewer)  // 世界坐标转经纬度
c2jwd(cartesian, viewer)              // cartesianToLnglat别名
cartesiansToLnglats(cartesians, viewer) // 世界坐标数组转经纬度数组
```

---

## 依赖关系图

```
App.js (HashRouter)
├── Home
│   ├── Header
│   └── useScrollReveal
├── Docs
│   ├── Header
│   ├── ReactMarkdown
│   └── docs (index.js)
├── About
│   ├── Header
│   └── @mikecousins/react-pdf
├── YqDistribution (Chart)
│   ├── Header
│   └── ECharts
├── Translate
│   ├── Header
│   └── api/getTranslate
├── ModelPreview
│   ├── Header
│   ├── ModelPreviewTab
│   │   ├── @react-three/fiber
│   │   ├── @react-three/drei
│   │   └── three (GLTFLoader)
│   └── SmartParkTab
│       ├── @react-three/fiber
│       ├── Scene
│       ├── DataPanel
│       └── Toolbar
├── LogicFlow
│   ├── Header
│   ├── @logicflow/core
│   └── @logicflow/extension
├── Video
│   └── Header
└── CesiumPage
    ├── Header
    ├── Cesium
    └── effects.js
        └── 30+ Effect Classes
```

---

## 构建与运行

### 安装依赖
```bash
npm install
```

### 开发模式
```bash
npm start
# 访问 http://localhost:3000
```

### 生产构建
```bash
npm run build
# 输出到 build/ 目录
```

### 测试
```bash
npm test
```

---

## 环境配置

### 关键环境变量
| 变量 | 说明 | 位置 |
|------|------|------|
| CESIUM_TOKEN | Cesium Ion访问令牌 | src/pages/cesium/constants.js |
| 彩云翻译Token | API认证令牌 | src/api/index.js |

### Cesium配置
```javascript
// src/pages/cesium/constants.js
CESIUM_TOKEN
INITIAL_CAMERA
VIEWER_OPTIONS
STATS
```

---

## 命名约定

| 类型 | 约定 | 示例 |
|------|------|------|
| 组件文件 | PascalCase | `ModelPreviewTab.jsx` |
| 工具函数 | camelCase | `getTranslate` |
| 样式文件 | index.scss/css | `index.scss` |
| 自定义Hook | use前缀 | `useScrollReveal` |
| 常量 | UPPER_SNAKE_CASE | `CESIUM_TOKEN` |
| 文档分类 | 中文 | "AI 工具" |

---

## 页面入口映射

```javascript
// src/App.js
<HashRouter>
  <Routes>
    <Route path="/" element={<Home />} />
    <Route path="/home" element={<Home />} />
    <Route path="/docs" element={<Docs />} />
    <Route path="/about" element={<About />} />
    <Route path="/chart" element={<Chart />} />
    <Route path="/translate" element={<Translate />} />
    <Route path="/preview" element={<ModelPreview />} />
    <Route path="/logicflow" element={<LogicFlow />} />
    <Route path="/video" element={<Video />} />
    <Route path="/cesium" element={<CesiumPage />} />
    <Route path="*" element={<NoFound />} />
  </Routes>
</HashRouter>
```

---

## 特性亮点

1. **3D可视化能力**: 整合Three.js (模型预览) + Cesium (地球) 双3D技术栈
2. **30+ Cesium特效**: 完整的WebGL特效系统（火焰、爆炸、闪电、烟花等）
3. **流程图编辑器**: 支持自定义节点样式的可视化编辑器
4. **文档系统**: 内置Markdown渲染、搜索、代码高亮
5. **响应式设计**: 移动端适配
6. **组件化架构**: 高度解耦的React组件设计
7. **工程化**: Webpack自定义配置、Jest测试、CI/CD集成

---

## 注意事项

1. Cesium Ion Token需要从 [cesium.com/ion](https://cesium.com/ion) 申请
2. 翻译API使用彩云科技服务，需申请Token
3. Cesium页面大量使用`useRef`缓存Cesium实体避免重渲染
4. 模型预览使用共享参数ref系统优化性能
5. 文档使用动态import `.md` 文件

---

*本文档由代码分析自动生成，如有不准确之处请参考源码*
