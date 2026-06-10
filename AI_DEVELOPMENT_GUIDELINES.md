# AI 开发规范文档 (myself 项目)

> 本文档专供所有 AI 助手（Claude / ChatGPT / Gemini / 豆包 等）在为这个项目生成或修改代码时阅读。
> 请在开始任何操作前，**先完整阅读一次**本文档，不遵守规则产生的代码视为无效。

---

## 0. 操作前必读：AI 工作流程

### 0.1 每次动手前的 4 步检查清单

```
[ ] 1. 我已经阅读完本文档了吗？
[ ] 2. 我是否明确了用户的需求？（不明确就问，不要猜）
[ ] 3. 我是否已经先读取了相关的现有代码文件？
[ ] 4. 我是否确认了改动方案不会破坏现有功能？
```

以上 4 条未全部勾选时，**不要写任何代码**。

### 0.2 阅读优先，代码其次

- 永远先 `Read` 现有文件理解上下文，再 `Edit`
- 改动复杂组件前，先列出你打算改什么、为什么，**在脑子里过一遍再输出**
- 当你不确定某个函数的作用时，**去读它**，不要假设

### 0.3 最小改动原则

- 能用 `Edit` 改 5 行，就不要用 `Write` 整文件覆盖
- 能用一个 CSS 类解决的，不要新建一个组件
- 能复用现有工具函数的，不要写新的
- 不触碰与任务无关的代码

### 0.4 先验证，再说"搞定了"

- 所有 JSX/JS 改动都要经过语法检查
- 涉及路由的改动，要检查 App.js 的路由映射
- 涉及样式的改动，要检查 class 命名是否符合命名规范（见下文）

---

## 1. 项目技术栈（写代码前必须确认）

**不要引入以下列表之外的新技术 / 新依赖**，除非用户明确要求。

| 领域 | 已采用技术 | 不要引入 |
|------|-----------|---------|
| 框架 | React 18 + 函数组件 + Hooks | Vue / Svelte / Angular / 类组件 |
| 路由 | react-router-dom v6 | Next.js / react-location |
| 3D (地球) | Cesium 1.141 | 其他地图库 |
| 3D (模型) | Three.js + @react-three/fiber + @react-three/drei | Babylon.js / 原始 three-raw |
| 流程图 | @logicflow/core + @logicflow/extension | React Flow |
| 图表 | ECharts 5.3 | Chart.js / D3 (除非极特殊场景) |
| 文档渲染 | react-markdown + remark-gfm + react-syntax-highlighter | MDX |
| 样式 | SCSS + 原生 CSS + BEM | CSS-in-JS (styled-components 等) + Tailwind 已装但不主导 |
| HTTP | axios | 不要换 fetch 封装 |
| 构建 | Webpack (Create React App 风格) | 不要换 Vite |
| 语言 | JavaScript (不是 TS) | 不要整文件转 TS |
| 状态 | React useState / useRef / useMemo | 不要引入 Redux / Zustand / Jotai |

### 为什么不让乱加？

- 本项目已经包含 `cesium` / `three` / `echarts` / `logicflow` 等重量级依赖，包体积已很大
- 保持构建链稳定，**不碰 webpack 配置**
- 代码风格已经成型，不做半吊子迁移

---

## 2. 命名规范（重中之重）

### 2.1 组件命名

```
✓ 好的: ModelPreviewTab, EffectLibraryPanel, CityPopup
✗ 坏的: tab, mp, preview_page, PreviewTabComponent (冗余后缀)
```

- **PascalCase**，语义清晰，一眼看懂
- 每个组件文件夹下放 `index.jsx` 作为主入口，样式文件 `index.scss` / `index.css`

### 2.2 函数与变量

```javascript
// ✓ 好的
function createProvinceLabels(viewer, geojson) { ... }
const filteredCategories = useMemo(() => ..., [categories, keyword]);
const isCurrentVisible = visibleItems.some(...);

// ✗ 坏的（不写缩写、不写无意义变量）
let d = [];                // d 是什么？
function process() { ... } // 处理什么？
const handleClick()  { ... } // 点哪里？
```

- **camelCase** 用于变量和函数
- 布尔变量用 `is/has/can/should` 前缀：`isLoading`, `hasData`, `canNavigate`
- 事件处理函数用 `handle` 前缀：`handleClose`, `handleBuildingClick`
- 不要用单字母变量名（循环的 `i` 可以接受，但 `item` 比 `it` 好）

### 2.3 CSS 类名

```scss
/* ✓ BEM 风格 - 看现有代码是这么写的 */
.wzt-button              /* Block */
.wzt-button--active      /* Modifier */
.home-main__title        /* Element */

/* ✓ 两个单词用横杠 */
.cesium-page, .effect-panel, .feature-section

/* ✗ 不要混用 */
.camelCaseInCss          /* 禁止 */
.underscore_style        /* 禁止，除了 BEM 的双下划线 __ */
```

- 组件根元素的 class = 组件名的 kebab-case，例如 `CesiumPage` → `cesium-page`
- 子元素用 `parent__child`
- 状态变体用 `item--state`

### 2.4 常量与配置

```javascript
// ✓ 大写下划线
const CESIUM_TOKEN = "xxx";
const INITIAL_CAMERA = { ... };
const EFFECT_TYPES = [ ... ];

// ✗ 不要
let token = "xxx";   // 魔术字符串
const arr = [ ... ]; // 语义不明
```

- 项目级常量放在 `constants.js` 或同目录的配置文件里
- 不要在组件内部写"魔术数字"，提取为命名常量

---

## 3. React 组件写作规范

### 3.1 文件结构模板

**每个页面组件** (`src/pages/xxx/index.jsx`):

```javascript
import React, { useState, useRef, useEffect, useCallback } from "react";
import Header from "../../components/Header";
import "./index.scss";

// —— 1. 静态配置数据（不要放 useState 里） ——
const FEATURE_ITEMS = [
  { text: "首页", path: "/home" },
];

// —— 2. Helper 函数（纯逻辑，无依赖） ——
function calculateReadingTime(text) {
  return Math.max(1, Math.ceil(text.length / 500));
}

// —— 3. 组件主体 ——
function PageName() {
  // 3.1 State 声明：同类放一起，注释分组
  const [data, setData] = useState([]);
  const [isLoading, setIsLoading] = useState(false);

  // 3.2 Ref 声明（Cesium Viewer / DOM / 缓存值）
  const viewerRef = useRef(null);

  // 3.3 Derived values (useMemo)
  const filtered = useMemo(
    () => data.filter(item => item.visible),
    [data]
  );

  // 3.4 回调 (useCallback)
  const handleClick = useCallback((id) => {
    // ...
  }, []);

  // 3.5 副作用 (useEffect) - 按执行顺序写
  useEffect(() => {
    // 初始化逻辑
    return () => { /* 清理 */ };
  }, []);

  // 3.6 JSX 返回
  return (
    <div className="page-name">
      <Header />
      <main>
        {/* JSX 内容 */}
      </main>
    </div>
  );
}

export default PageName;
```

### 3.2 Cesium/Three 组件的特殊规范

**Cesium 组件必须管理生命周期**：

```javascript
// ✓ 必须这么写
useEffect(() => {
  const viewer = new Cesium.Viewer(containerRef.current, OPTIONS);
  viewerRef.current = viewer;

  // 注册的事件处理器 / 定时器 —— 必须保存引用

  return () => {
    // 组件卸载时 —— 必须销毁一切
    mouseMoveHandler?.destroy();
    clickHandler?.destroy();
    window.clearInterval(timerId);
    viewerRef.current?.destroy();
    viewerRef.current = null;
  };
}, []);

// ✗ 永远不要这么写（内存泄漏 + 重复初始化）
useEffect(() => {
  new Cesium.Viewer(...); // 没有 return 清理
});
```

**Three.js (react-three/fiber) 组件**：

- 使用 `@react-three/drei` 提供的现成组件（OrbitControls, useProgress 等）
- 参数共享使用 module-level ref（见 `ModelPreviewTab` 的 `sharedParamsRef` 模式），避免不必要的 re-render
- 不要在 `useFrame` 里 setState，每帧 setState = 性能灾难

### 3.3 导入顺序

```javascript
// 1. React 核心
import React, { useState, useEffect } from "react";

// 2. 第三方库（字母顺序或按类别）
import { HashRouter, Route, Routes } from "react-router-dom";
import * as Cesium from "cesium";
import * as echarts from "echarts";

// 3. 本项目组件
import Header from "../../components/Header";
import CityPopup from "./components/CityPopup";

// 4. 工具函数 / API
import { getTranslate } from "../../api";

// 5. 数据 / 配置
import { CITIES_DATA } from "./cityData";
import { CESIUM_TOKEN } from "./constants";

// 6. 样式（最后一行）
import "./index.scss";
import "cesium/Build/Cesium/Widgets/widgets.css";
```

**不要使用相对路径 "../" 超过 3 层**，如果发现要 `../../../components/...`，说明你可能在嵌套过深的目录里，或者应该用路径别名（暂未配置，写完整路径即可）。

---

## 4. 样式规范

### 4.1 用 SCSS 还是 CSS？

- 已有 SCSS 文件的目录继续用 SCSS
- 简单页面用 CSS 也可以
- **不要混用一个组件里两个文件做同样的事**

### 4.2 样式作用域

```scss
/* ✓ 好的 - 用组件根 class 做命名空间 */
.cesium-page {
  .cesium-main { ... }
  .info-panel { ... }
}

/* ✗ 坏的 - 全局污染 */
.button { color: red; }      /* 其他页面的 button 也会变红 */
.panel  { background: #fff; } /* 所有 panel 被影响 */
```

### 4.3 颜色与间距

- 不要写"随机色值"如 `#1a2b3c`，如果多处复用，应该提取变量
- 本项目尚未引入 CSS 变量体系，**如你想统一颜色请先跟用户确认方案**，不要自己搞一套

---

## 5. 路由规范

### 5.1 新增页面的标准流程

1. 在 `src/pages/your-page/index.jsx` 创建组件
2. 在 `src/App.js` 的 `<Routes>` 里加一条 `<Route>`
3. 在 `src/components/Header/index.jsx` 的 `navItems` 数组里加一项（如果需要出现在导航栏）
4. 页面内部必须包含 `<Header />`（除非是弹窗类页面）

### 5.2 路径约定

```javascript
// ✓ 全小写，连字符
<Route path="/model-preview" element={<ModelPreview />} />

// ✗ 不要大写、不要下划线
<Route path="/ModelPreview" ... />   // 禁止
<Route path="/model_preview" ... />   // 禁止
```

### 5.3 不要用 window.location.href

```javascript
// ✓ 用 React Router 的方式
import { useNavigate } from "react-router-dom";
const navigate = useNavigate();
navigate("/preview");

// ✗ 不要（会整页刷新，丢失所有状态）
window.location.href = "/preview";
```

---

## 6. Cesium 模块开发规范

### 6.1 特效类的标准写法

文件位置：`src/pages/cesium/effect/YourEffectNameEffect.js`

```javascript
// 命名规则: XxxEffect
class CircleWarnEffect {
  constructor(viewer, options) {
    // 解构 options + 默认值
    const {
      longitude = 0,
      latitude = 0,
      height = 0,
      radius = 100000,
      speed = 1.0,
      color = Cesium.Color.RED,
      autoAnimate = true,
      Cesium: CesiumRef   // 从外部注入 Cesium 引用
    } = options;

    this.viewer = viewer;
    this.entities = [];

    // ... 创建 entity 逻辑

    if (autoAnimate) {
      this._startAnimation();
    }
  }

  // 私有方法用 _ 前缀
  _startAnimation() { ... }
  _updateMaterial() { ... }

  // 必须提供销毁方法
  destroy() {
    this.entities.forEach(e => this.viewer.entities.remove(e));
  }
}

export default CircleWarnEffect;
```

### 6.2 在 `effects.js` 注册新特效

- 在 `EFFECT_TYPES` 数组里加一条 `{ key, label, color }`
- 在 `createWebGLEffect` 的 `entityFactories` 里加对应的 factory 函数
- 在 `effect/` 目录下新增对应的 `XxxEffect.js` 类

### 6.3 坐标系统

- Cesium 内部用 Cartesian3，不要在 JSX 里传 Cartesian3 做 prop
- 经纬度展示用 `Cesium.Math.toDegrees()` 转成数字
- **永远不要假设地球半径是 6371km**，用 Cesium 自己的常量

---

## 7. 文档与 Markdown 规范

### 7.1 新增文档

在 `src/docs/` 下创建 `.md` 文件，然后在 `src/docs/index.js` 的 `docs` 数组登记：

```javascript
{
  title: "文档标题",
  category: "前端基础",      // 从现有分类里选，不要乱加新分类
  tags: ["React", "Hooks"],  // 控制在 1-5 个标签
  value: importedMarkdown
}
```

### 7.2 Markdown 内容规范

- 标题层级最多 3 级（`#` / `##` / `###`）
- 代码块必须指定语言：\`\`\`javascript ...
- 正文用中文，术语可以用英文
- 新文件顶部不要加 YAML front matter（本项目不支持）

---

## 8. API 与数据获取规范

### 8.1 在哪里放 API 调用

- **跨页面复用的 API** → `src/api/index.js`
- **页面内一次性调用** → 放在页面目录的 `api.js`（如果后续需要）
- **不要在组件里裸写 axios 调用 + URL**，至少封装一层

### 8.2 当前唯一 API: 翻译

```javascript
import { getTranslate } from "../../api";

// 返回 Promise，.data.target 是翻译结果
const res = await getTranslate(text, language);
```

- 不要改动这个 API 的签名（除非用户要求）
- Token 硬编码在文件里，本项目是公开 demo 没问题
- 真实生产项目应该放后端代理，**不要在新项目里照抄这个模式**

### 8.3 不要 mock 真实功能

- 不要因为"懒得接 API"就写假数据代替真实功能
- 做不出来就如实告诉用户，提替代方案
- 写 `// TODO: 接入后端` 然后返回硬编码数组 = 不合格

---

## 9. 性能与代码质量

### 9.1 必须使用 memo 优化的场景

```javascript
// ✓ 列表项有独立交互时
<List>
  {items.map(item => (
    <ItemRow key={item.id} data={item} onClick={handleItemClick} />
  ))}
</List>
// 注：ItemRow 如果计算很重，用 React.memo 包一层
```

### 9.2 不要在 render 里做重计算

```javascript
// ✗ 坏的 - 每次 render 都重新过滤 + 排序 + 映射
const visible = items.filter(i => i.active).sort(...).map(...)

// ✓ 好的 - 用 useMemo
const visible = useMemo(
  () => items.filter(i => i.active).sort(...).map(...),
  [items]
);
```

### 9.3 Cesium 性能要点

- Viewer 必须只初始化一次（StrictMode 下用 ref guard）
- 事件处理器必须 destroy
- 实体（Entity）不要每帧新建，复用 + 更新属性
- **禁止在 useEffect 里 setState 触发循环更新**

---

## 10. 绝对禁止清单

以下行为一经发现，生成的代码视为垃圾代码，不接受。

| 编号 | 禁止事项 | 后果 |
|------|---------|------|
| ❌1 | 引入 `package.json` 已有列表之外的新依赖 | 构建可能失败、体积膨胀 |
| ❌2 | 把 JS 文件整体改成 TS | 语法不一致，CI 失败 |
| ❌3 | 用 `Write` 整文件覆盖已有代码 | 容易引入语法错误（历史教训） |
| ❌4 | 在 render 路径里 new Cesium.Viewer / new THREE.Scene | 内存泄漏 + 性能爆炸 |
| ❌5 | 在 JSX 里用 `dangerouslySetInnerHTML` 渲染用户输入 | XSS 风险 |
| ❌6 | 用 `window.location.href` 做 SPA 跳转 | 整页刷新、状态丢失 |
| ❌7 | 写无类型的 `any` 风格变量 `let data = ...` 且不注释 | 没人看得懂 |
| ❌8 | 在 CSS 里写裸 class `.button` `.panel` | 全局样式污染 |
| ❌9 | 把 API token / secret 提交到代码里（除本项目公开 demo token 外） | 安全问题 |
| ❌10 | 自己"想象"一套功能代替真实实现 | 浪费时间 |
| ❌11 | 改动 App.js 的路由映射后忘记同步 Header 导航 | 用户点不到新页面 |
| ❌12 | useEffect 不写 cleanup 返回值（特别涉及 Cesium/Three） | 内存泄漏 |
| ❌13 | npm 包版本号写 `*` 或 `latest` | 破坏可复现构建 |
| ❌14 | 组件函数超过 800 行还不拆 | 不可维护 |
| ❌15 | 连续 3 次同样的改动还没跑通还继续改 | 陷入死循环，应该停下来总结问题 |

---

## 11. 当你不知道怎么做时

### 11.1 先找参考

本项目里**已经有很多模式可以抄**，不要自己发明：

| 你想做的事 | 去看这个文件 |
|-----------|-------------|
| 新增页面路由 | [App.js](file:///d:/work/resourse/myself/src/App.js) |
| 写新页面模板 | [home/index.jsx](file:///d:/work/resourse/myself/src/pages/home/index.jsx) |
| 写 Cesium 特效 | [effects.js](file:///d:/work/resourse/myself/src/pages/cesium/effects.js) |
| 写 Three.js 模型页 | [ModelPreviewTab.jsx](file:///d:/work/resourse/myself/src/pages/model-preview/ModelPreviewTab.jsx) |
| 写流程图节点 | [logicflow/nodeStyles/](file:///d:/work/resourse/myself/src/pages/logicflow/nodeStyles/) |
| 写 ECharts 图表 | [yq-distribution/index.jsx](file:///d:/work/resourse/myself/src/pages/yq-distribution/index.jsx) |
| 写组件按钮 | [wzt-button/index.jsx](file:///d:/work/resourse/myself/src/components/wzt-button/index.jsx) |
| 写工具函数 | [utils/index.js](file:///d:/work/resourse/myself/src/utils/index.js) |
| 写 Hook | [useScrollReveal.js](file:///d:/work/resourse/myself/src/hooks/useScrollReveal.js) |
| 注册文档 | [docs/index.js](file:///d:/work/resourse/myself/src/docs/index.js) |

### 11.2 向用户提问的正确姿势

如果你拿不准：

```
我的分析：
  1. 现有代码用 XXX 模式实现（见 file.js 第 20 行）
  2. 如果按你的需求做，有两种实现方案
     方案 A: ...（优点/缺点）
     方案 B: ...（优点/缺点）
我的建议：方案 A，因为 ...
请问你选哪个？
```

**不要沉默执行可能有争议的方案**。

### 11.3 自我审查清单（输出代码前过一遍）

```
[ ] 1. 所有 import 路径都正确吗？（注意 ../ 的层级）
[ ] 2. JSX 每个标签都闭合了吗？
[ ] 3. 每个 useEffect 有 cleanup 吗（如果涉及外部资源）
[ ] 4. map 循环有 key 吗？
[ ] 5. 样式用了命名空间 class 吗？
[ ] 6. 新依赖加了吗（不允许加的话就不要加）
[ ] 7. 有没有改写了和任务无关的代码？
[ ] 8. 命名能让 3 个月后的自己看懂吗？
[ ] 9. 有没有中文注释解释"为什么"，而不是"做了什么"？
[ ]10. 这段代码在 StrictMode 下运行两次会出问题吗？
```

---

## 12. 给 AI 的特别提醒

### 12.1 你不是在白纸上写项目

- 这个项目已经有成型的风格和依赖
- 你的任务是**融入**而不是**改造**
- 改动 5 行就能解决的问题，不要输出 50 行

### 12.2 不要过度设计

- 用户说"加一个按钮" → 就加一个按钮
- 不要顺手"改造一下整体布局"、"顺便抽出一个高阶组件"、"重构一下状态管理"
- **大的改动单独提 PR，单独讨论**

### 12.3 你可以提出改进建议，但先说清楚"这是建议"

- 发现潜在问题可以提
- 但不要未经确认就改代码
- 格式：`⚠️ 建议：我注意到 xxx 处可能有 yyy 问题，是否需要处理？`

### 12.4 遇到自己不会的库

- **如实告知**：这个库我不熟悉，需要读一下现有代码/文档才能动手
- 不要瞎猜 API，Cesium 和 Three.js 的 API 记错了就是硬伤

---

## 13. 示例：一个"合格"的改动

**用户需求**: 在 Cesium 页面加一个"隐藏所有特效"按钮

**AI 应该怎么做**:

1. ✅ 读 [cesium/index.jsx](file:///d:/work/resourse/myself/src/pages/cesium/index.jsx) 理解 `webGLEffectsRef` 结构
2. ✅ 读 [cesium/effects.js](file:///d:/work/resourse/myself/src/pages/cesium/effects.js) 的 `removeWebGLEffect`
3. ✅ 在 CesiumPage 里加一个 `clearAllEffects` 函数（复用 `removeWebGLEffect` 循环）
4. ✅ 在 feature-panel 里加一个 button
5. ✅ 样式用已有的 `feature-panel__actions` 类命名空间
6. ✅ 输出代码改动（小范围 Edit，不是 Write 整文件）

**AI 不应该做**:

- ❌ 把整个 Cesium 组件重构成 TypeScript
- ❌ 顺手把 feature-panel 拆成 5 个组件
- ❌ 引入 react-cesium 第三方封装
- ❌ 把按钮样式改成 Tailwind（虽然装了，但本项目不用它写样式）

---

## 14. 结束前：自我验证 Checklist

在你认为"做完了"之前，最后过一遍：

```
[ ] 运行 npm start 没有控制台报错（如果不能运行，至少语法自查）
[ ] 改动的页面在浏览器里点一遍能正常工作
[ ] 新增的路由在 Header 里（如果需要）
[ ] 没有引入新依赖
[ ] 没有破坏其他页面的样式
[ ] 命名符合规范
[ ] Cesium/Three 资源在组件卸载时正确清理
[ ] 代码里没有 TODO: "以后补" 的核心逻辑
[ ] 改动量是解决问题所需的最小值
[ ] 用户能看懂你写了什么（如果你自己看一遍都觉得绕，就重写）
```

---

> **最后一句话**:
> 这个项目的价值在于"能跑起来、能看、能玩"，不是写一本教科书。
> 实用优先，优雅其次。写简单的好代码，胜过写复杂的"聪明"代码。
