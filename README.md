# myself - 个人技术实验室

> 展示前端工程实践、3D 可视化、流程图编辑与技术文档的个人作品集。

**在线访问**: https://wangzt-arch.github.io/myself

---

## 技术栈

| 领域 | 技术 |
|------|------|
| 框架 | React 18 + React Router v6 |
| 3D 地球 | Cesium 1.141 |
| 3D 模型 | Three.js + @react-three/fiber + @react-three/drei |
| 流程图 | @logicflow/core + @logicflow/extension |
| 图表 | ECharts 5.3 |
| 文档渲染 | react-markdown + react-syntax-highlighter |
| 样式 | SCSS + CSS Modules |
| 构建 | Webpack 5 (CRA 风格) |

---

## 功能页面

| 路由 | 页面 | 说明 |
|------|------|------|
| `/home` | 首页 | 功能导航入口 |
| `/about` | 关于 | PDF 简历预览 |
| `/docs` | 文档中心 | Markdown 技术文档，支持搜索与代码高亮 |
| `/chart` | 数据可视化 | ECharts 驾驶舱：地图、折线、柱状、饼图等 |
| `/translate` | 翻译工具 | 彩云翻译 API 接入，中英互译 |
| `/preview` | 3D 模型预览 | GLTF 模型查看器，支持材质调节与爆炸图 |
| `/logicflow` | 流程图编辑 | LogicFlow 可视化编辑器，自定义节点样式 |
| `/video` | 视频案例 | 视频素材瀑布流展示 |
| `/cesium` | 3D 地球 | Cesium 地球可视化，包含 30+ WebGL 特效 |

---

## 快速开始

```bash
# 安装依赖
npm install

# 开发模式（http://localhost:3000）
npm start

# 生产构建
npm run build
```

---

## 提交规范

本项目使用 **commitlint + commitizen** 强制提交信息格式，不符合规范的提交将被 Git hooks 拦截。

### 格式

```
type(scope): 中文描述
```

### type 可选值

| type | 含义 |
|------|------|
| `feat` | 新功能 / 新增页面 / 新增组件 |
| `fix` | 修复 bug |
| `style` | 代码格式调整（不影响逻辑） |
| `refactor` | 代码重构（非新功能也非修 bug） |
| `perf` | 性能优化 |
| `docs` | 文档变更 |
| `test` | 新增或调整测试 |
| `build` | 构建工具 / 依赖升级 / 脚手架变更 |
| `ci` | CI/CD 配置变更 |
| `chore` | 杂项（.gitignore / 配置等） |
| `revert` | 回滚之前的提交 |

### 提交方式

**方式一：交互式提交（推荐）**

```bash
npm run commit
# 按提示选择 type → scope → 填写中文描述
```

**方式二：手工提交**

```bash
git commit -m "feat(cesium): 新增城市点击弹窗"
```

> ⚠️ 提交信息**必须包含中文描述**，否则会被 commit-msg hook 拦截拒绝。

---

## 项目结构

```
src/
├── api/                 # API 接口封装
├── components/          # 通用组件（Header、Loading、OpenScreen 等）
├── docs/                # Markdown 技术文档（21 篇）
├── hooks/               # 自定义 Hooks
├── pages/               # 页面组件
│   ├── home/            # 首页
│   ├── about/           # PDF 简历
│   ├── docs/            # 文档中心
│   ├── cesium/          # Cesium 3D 地球 + 30+ 特效系统
│   ├── model-preview/   # 3D 模型预览 / 智慧园区
│   ├── logicflow/       # 流程图编辑器
│   ├── yq-distribution/ # ECharts 数据可视化
│   ├── translate/       # 翻译工具
│   └── video/           # 视频案例
├── utils/               # 工具函数
├── App.js               # 路由配置
└── index.js             # 入口文件
```

---

## 相关文档

- [Code Wiki](CODE_WIKI.md) — 项目架构、模块说明、依赖关系
- [AI 开发规范](AI_DEVELOPMENT_GUIDELINES.md) — AI 协作代码规范

---

## 注意事项

- Cesium Ion Token 需从 [cesium.com/ion](https://cesium.com/ion) 申请后替换 `src/pages/cesium/constants.js` 中的占位值
- 翻译工具使用彩云科技 API，Token 位于 `src/api/index.js`
- 部署至 GitHub Pages：`npm run build`，产物输出至 `build/` 目录
