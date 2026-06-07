# 前端工程化面试题

> 前端工程化相关的高频面试题，涵盖构建工具（Webpack/Vite/Rollup）、代码规范、性能优化、CI/CD，以口语化回答为主，由浅入深排列。

---

## 一、基础概念

### 1.1 什么是前端工程化？

前端工程化就是把软件工程的方法和思想应用到前端开发中，让开发流程更规范、更高效、更可靠。

具体来说，前端工程化包括这几个方面：

**代码规范**：统一代码风格，比如用 ESLint 检查 JS 代码规范、Prettier 自动格式化代码、Stylelint 检查 CSS 规范。这样团队协作时代码风格一致，减少不必要的代码审查争论。

**构建工具**：用 Webpack、Vite、Rollup 等工具把源代码（ES6+、TypeScript、SCSS、Vue/React 组件等）编译打包成浏览器能识别的代码，同时进行代码压缩、Tree Shaking、代码分割等优化。

**自动化测试**：单元测试（Jest、Vitest）、E2E 测试（Cypress、Playwright），保证代码质量，减少线上 bug。

**持续集成/持续部署（CI/CD）**：代码提交后自动跑测试、自动构建、自动部署，减少人工操作，降低出错概率。

**版本管理与发布**：用 Git 管理代码版本，用 npm 管理依赖版本，用 Changesets 或 Semantic Versioning 规范版本号管理。

---

### 1.2 模块规范有哪些？CommonJS 和 ES Module 的区别？

前端模块规范主要有 CommonJS、AMD、UMD 和 ES Module 这几种。

**CommonJS** 是 Node.js 使用的模块规范，用 `require` 导入、`module.exports` 导出。它是同步加载的，适合服务端，因为服务端文件都在本地，读取速度快。

**AMD** 是浏览器端的异步模块规范，RequireJS 就是基于 AMD 的，用 `define` 定义模块、`require` 异步加载。现在基本已经被淘汰了。

**UMD** 是一种通用模块规范，同时兼容 CommonJS、AMD 和全局变量，很多第三方库（如 jQuery、Lodash）都用 UMD 格式发布。

**ES Module** 是 ES6 引入的官方模块规范，用 `import` 导入、`export` 导出。它是静态的，编译时就能确定依赖关系，支持 Tree Shaking。现代浏览器原生支持，Node.js 12+ 也支持。

| 特性 | CommonJS | ES Module |
|------|----------|-----------|
| 语法 | `require` / `module.exports` | `import` / `export` |
| 加载方式 | 运行时同步加载 | 编译时静态分析 |
| 动态导入 | 直接 `require(path)` | `import()` 函数 |
| Tree Shaking | 不支持 | 支持 |
| 循环依赖 | 支持 | 支持（但行为不同） |
| 浏览器支持 | 不支持（需打包） | 原生支持 |

实际项目中，源码用 ES Module 写，打包工具会把 ES Module 编译成浏览器能运行的代码。

---

### 1.3 npm 包版本号规则是什么？

npm 使用**语义化版本控制（Semantic Versioning）**，版本号格式是 `主版本.次版本.修订号`，比如 `1.2.3`。

- **主版本（Major）**：做了不兼容的 API 修改，升级后可能需要修改代码
- **次版本（Minor）**：新增了向下兼容的功能
- **修订号（Patch）**：修复了 bug，向下兼容

**版本范围符号**：

- `^1.2.3`：兼容次版本和修订号更新，即 `>=1.2.3 <2.0.0`，会自动升级到 `1.x.x` 的最新版本
- `~1.2.3`：只兼容修订号更新，即 `>=1.2.3 <1.3.0`，会自动升级到 `1.2.x` 的最新版本
- `1.2.3`：固定版本，不会自动更新
- `*`：任意版本
- `>1.2.3`、`>=1.2.3`、`<2.0.0`：指定范围

`package-lock.json` 或 `yarn.lock` 的作用是锁定依赖的确切版本，保证不同环境安装的依赖版本一致，避免"在我电脑上能跑"的问题。

---

## 二、构建工具

### 2.1 Webpack 的核心概念？

Webpack 是目前最主流的模块打包工具，它的核心概念有：

**Entry（入口）**：指定打包从哪个文件开始，可以配置单入口或多入口。

**Output（输出）**：指定打包后的文件输出到哪里，包括文件名、路径等。

**Loader（加载器）**：Webpack 本身只认识 JS 和 JSON，Loader 的作用是把其他类型的文件（CSS、图片、TypeScript、Vue 组件等）转换成 Webpack 能处理的模块。

常用的 Loader 有：`babel-loader`（转译 ES6+）、`css-loader`（处理 CSS 中的 `@import` 和 `url`）、`style-loader`（把 CSS 注入 DOM）、`sass-loader`（编译 SCSS）、`file-loader` / `url-loader`（处理图片字体等静态资源）、`ts-loader`（编译 TypeScript）。

**Plugin（插件）**：在打包的各个环节执行额外的任务，功能比 Loader 更强大。

常用的 Plugin 有：`HtmlWebpackPlugin`（自动生成 HTML 并注入打包后的资源）、`CleanWebpackPlugin`（清理输出目录）、`MiniCssExtractPlugin`（提取 CSS 为单独文件）、`DefinePlugin`（定义全局常量）、`HotModuleReplacementPlugin`（热更新）。

**Mode（模式）**：`development` 或 `production`，不同模式会启用不同的默认优化。

**Module（模块）**：Webpack 把一切文件都视为模块，通过 Loader 转换后统一处理。

**Chunk（代码块）**：Webpack 打包过程中生成的代码块，一个 Chunk 对应一个或多个输出文件。

**Bundle（打包文件）**：最终输出的文件，一个 Bundle 对应一个 Chunk。

---

### 2.2 Webpack 的构建流程是怎样的？

Webpack 的构建流程大致分为这几个阶段：

**初始化阶段**：读取配置文件，合并默认配置，创建 Compiler 对象。

**编译阶段**：
1. 从 Entry 开始，递归解析模块依赖
2. 对每个模块，用匹配的 Loader 进行转换
3. 解析模块中的 `import` / `require`，找到依赖的模块，继续递归处理

**生成阶段**：
1. 根据模块依赖关系构建 Chunk Graph
2. 对每个 Chunk 进行代码优化（Tree Shaking、代码压缩、Scope Hoisting 等）
3. 生成最终的打包文件（Bundle）

**输出阶段**：把 Bundle 写入到 Output 指定的目录。

整个过程中，Plugin 可以在各个生命周期钩子中介入，执行自定义逻辑。

---

### 2.3 Webpack 中 Loader 和 Plugin 的区别？

**Loader** 是**文件转换器**，它的作用是把某种类型的文件转换成另一种类型，让 Webpack 能处理。Loader 是在模块加载时执行的，一个文件可能经过多个 Loader 的链式处理。

**Plugin** 是**任务执行器**，它的作用是在打包的某个生命周期阶段执行额外的任务，功能更灵活。Plugin 可以访问 Webpack 的内部机制，在整个构建过程中都能介入。

简单说：Loader 处理的是**单个文件的内容转换**，Plugin 处理的是**整个构建流程**。

---

### 2.4 什么是 Tree Shaking？Webpack 如何实现？

Tree Shaking 也叫"摇树优化"，它的作用是**把代码中没有用到的部分自动剔除**，减少打包体积。

原理是基于 ES Module 的**静态结构**。因为 ES Module 的 `import` / `export` 是在编译时就能确定的，不像 CommonJS 的 `require` 是运行时动态执行的。所以打包工具可以静态分析出哪些导出被使用了，哪些没有被使用，然后把没用到的代码删除。

**Webpack 开启 Tree Shaking 的条件**：

1. 使用 ES Module（`import` / `export`）
2. 配置 `mode: 'production'`（生产模式默认开启）
3. 在 `package.json` 中设置 `"sideEffects": false`（告诉 Webpack 没有副作用，可以安全删除未使用的代码）

**注意**：如果某些文件有副作用（比如 polyfill、全局样式），需要在 `sideEffects` 中声明，否则会被误删。

```json
{
  "sideEffects": [
    "*.css",
    "*.scss",
    "./src/polyfill.js"
  ]
}
```

---

### 2.5 Webpack 的代码分割（Code Splitting）有哪些方式？

代码分割就是把一个大包拆成多个小包，按需加载，减少首屏加载时间。

**三种分割方式**：

**入口分割（Entry Points）**：配置多个入口，每个入口生成独立的 Chunk。

```javascript
module.exports = {
  entry: {
    main: './src/index.js',
    vendor: './src/vendor.js',
  },
};
```

**动态导入（Dynamic Imports）**：用 `import()` 函数按需加载模块，Webpack 会自动把动态导入的模块拆成单独的 Chunk。

```javascript
// 点击按钮时才加载图表库
button.addEventListener('click', async () => {
  const echarts = await import('echarts');
  // 使用 echarts
});
```

**SplitChunksPlugin**：自动提取公共代码，比如多个入口共用的第三方库。

```javascript
optimization: {
  splitChunks: {
    chunks: 'all',
    cacheGroups: {
      vendor: {
        test: /[\\/]node_modules[\\/]/,
        name: 'vendors',
        chunks: 'all',
      },
    },
  },
}
```

---

### 2.6 Webpack 的热更新（HMR）原理？

热更新（Hot Module Replacement）就是在开发过程中，修改代码后页面不刷新，只更新变化的部分，保持应用状态不丢失。

**原理**：

1. 启动开发服务器时，Webpack 会注入 HMR Runtime 代码到打包文件中
2. 文件修改后，Webpack 重新编译变化的模块
3. 通过 WebSocket 或 EventSource 向浏览器推送更新消息
4. 浏览器收到消息后，通过 HMR Runtime 向服务器请求更新的模块
5. HMR Runtime 用新模块替换旧模块，执行模块的 `module.hot.accept` 回调

**配置**：

```javascript
// webpack.config.js
devServer: { hot: true },
plugins: [new webpack.HotModuleReplacementPlugin()],
```

**注意**：HMR 只更新 JS 模块，CSS 的 HMR 是 `style-loader` 或 `MiniCssExtractPlugin` 自动处理的。React/Vue 组件的 HMR 需要额外的 loader（如 `react-refresh-webpack-plugin`、`vue-loader`）。

---

### 2.7 Vite 和 Webpack 的区别？

Vite 是新一代的前端构建工具，和 Webpack 的设计理念有很大不同。

**开发阶段**：

- **Webpack**：先打包所有模块，生成 Bundle，然后启动开发服务器。项目越大，冷启动越慢。
- **Vite**：利用浏览器原生 ES Module 支持，直接按请求提供模块，不需要预先打包。冷启动几乎瞬间完成，按需编译，速度非常快。

**生产构建**：

- **Webpack**：用自己的打包逻辑生成 Bundle
- **Vite**：用 Rollup 打包，生成的代码更精简

| 特性 | Webpack | Vite |
|------|---------|------|
| 开发启动速度 | 慢（需先打包） | 快（原生 ESM，按需编译） |
| 热更新速度 | 中等（重新编译模块） | 快（ESM 按需更新） |
| 配置复杂度 | 较复杂，需要较多配置 | 较简单，约定优于配置 |
| 生态成熟度 | 非常成熟，插件丰富 | 较新，但发展迅速 |
| 生产打包 | 自己实现 | 基于 Rollup |
| 适用场景 | 大型项目、复杂需求 | 中小型项目、快速开发 |

**选择建议**：新项目优先用 Vite，启动快、配置简单；大型项目或需要复杂自定义配置时用 Webpack。

---

### 2.8 Rollup 是什么？适合什么场景？

Rollup 是一个 JavaScript 模块打包器，和 Webpack 相比，它更专注于打包 JavaScript 库（Library），而不是应用（Application）。

**Rollup 的特点**：

- **输出更精简**：Rollup 默认使用 ES Module 格式，打包后的代码更干净，没有 Webpack 的模块加载器代码
- **Tree Shaking 更彻底**：基于 ES Module 的静态分析，能更好地剔除未使用的代码
- **不支持 HMR**：没有热更新功能，不适合开发阶段使用
- **对代码分割支持较弱**：不如 Webpack 灵活

**适用场景**：
- 打包 JavaScript 库（如 Vue、React、Lodash 等库本身都是用 Rollup 打包的）
- 打包需要发布到 npm 的组件库
- 需要生成多种格式（ESM、CJS、UMD、IIFE）的库

**实际用法**：很多项目用 Vite 做开发服务器（利用 Vite 的快速启动），用 Rollup 做生产打包（Vite 生产构建底层就是 Rollup）。

---

### 2.9 Webpack、Vite、Rollup 如何选择？

| 工具 | 最佳场景 | 不太适合 |
|------|---------|---------|
| **Webpack** | 大型应用、复杂配置需求、需要丰富插件生态 | 追求极速启动 |
| **Vite** | 中小型应用、快速开发、现代浏览器项目 | 需要兼容 IE、超大型项目 |
| **Rollup** | 打包 JS 库、组件库、需要输出多种模块格式 | 应用开发、需要 HMR |

实际项目中，**应用开发**优先选 Vite 或 Webpack，**库开发**优先选 Rollup。很多现代项目采用组合方案：开发用 Vite，生产用 Rollup（Vite 底层）。

---

## 三、性能优化

### 3.1 前端性能优化有哪些手段？

前端性能优化可以从**加载**、**渲染**、**运行时**三个层面来考虑。

**加载优化**：
- 代码分割和懒加载，首屏只加载必要代码
- Tree Shaking 去掉未使用的代码
- 图片压缩，使用 WebP 格式，小图标用 SVG
- 静态资源上 CDN
- 开启 Gzip/Brotli 压缩
- 合理设置缓存策略

**渲染优化**：
- CSS 放 `head`，JS 放 `body` 底部或用 `defer`/`async`
- 减少 DOM 操作，批量修改样式
- 用 `transform` 和 `opacity` 做动画，触发 GPU 加速
- 长列表用虚拟滚动
- 防抖节流处理高频事件

**运行时优化**：
- 减少不必要的组件重渲染（React 中用 `memo`、`useMemo`）
- 大数据量计算用 Web Worker
- 避免内存泄漏，及时清理定时器和事件监听

---

### 3.2 如何分析打包体积？

**Webpack Bundle Analyzer**：可视化分析打包结果，直观看到每个模块占多大体积。

```javascript
const BundleAnalyzerPlugin = require('webpack-bundle-analyzer').BundleAnalyzerPlugin;
module.exports = {
  plugins: [new BundleAnalyzerPlugin()],
};
```

**Vite 的 `rollup-plugin-visualizer`**：类似功能，生成可视化的打包分析报告。

**分析后的优化方向**：
- 检查是否有重复依赖（比如同时引入了 lodash 和 lodash-es）
- 检查是否有大体积的第三方库，考虑按需引入或替换
- 检查是否有不应该被打包的文件（如测试文件、文档）

---

### 3.3 什么是按需加载（Lazy Loading）？

按需加载就是**在需要的时候才加载对应的代码**，而不是一开始就全部加载。

**路由级按需加载**：

```javascript
// React
const About = lazy(() => import('./pages/About'));

// Vue
const About = () => import('./pages/About.vue');
```

**组件级按需加载**：

```javascript
// 点击弹窗时才加载弹窗组件
async function openModal() {
  const { Modal } = await import('./components/Modal');
  Modal.open();
}
```

**库级按需加载**：

```javascript
// 不用 import _ from 'lodash'（全量引入）
// 而是按需引入
import debounce from 'lodash/debounce';
import throttle from 'lodash/throttle';

// 或者用 babel-plugin-import 自动按需引入
```

---

## 四、代码规范与质量

### 4.1 ESLint 和 Prettier 有什么区别？

**ESLint** 是**代码检查工具**，主要检查代码中的潜在问题，比如未使用的变量、可能的类型错误、不符合规范的写法等。它关注的是**代码质量和潜在 bug**。

**Prettier** 是**代码格式化工具**，主要处理代码的排版风格，比如缩进、换行、引号、分号等。它关注的是**代码风格的一致性**。

两者的分工不同：ESLint 管"写得对不对"，Prettier 管"排得整不整齐"。

实际项目中两者配合使用，用 ESLint 检查代码质量，用 Prettier 统一代码格式。为了避免冲突，可以用 `eslint-config-prettier` 关闭 ESLint 中与格式相关的规则，把格式工作完全交给 Prettier。

---

### 4.2 Git 工作流有哪些？

常见的 Git 工作流有：

**Git Flow**：最经典的工作流，有明确的分支模型。
- `main` / `master`：生产分支
- `develop`：开发分支
- `feature/*`：功能分支，从 develop 切出，完成后合并回 develop
- `release/*`：发布分支，从 develop 切出，测试完成后合并到 main 和 develop
- `hotfix/*`：热修复分支，从 main 切出，修复后合并到 main 和 develop

适合有固定发布周期的项目，但分支较多，管理较复杂。

**GitHub Flow**：简化版，只有 `main` 分支和 `feature` 分支。
- 从 `main` 切出功能分支
- 开发完成后发 Pull Request
- Code Review 通过后合并到 `main`
- 合并后立即部署

适合持续部署的项目，简单高效。

**GitLab Flow**：结合 Git Flow 和 GitHub Flow，增加了环境分支（如 `pre-production`、`production`）。

**Trunk-Based Development**：所有开发都在 `main` 分支上进行，用 Feature Toggle 控制功能开关。适合有完善自动化测试和持续部署的团队。

---

### 4.3 什么是 Monorepo？有什么优缺点？

Monorepo 就是**把多个相关的项目放在同一个代码仓库中管理**。

比如一个前端项目可能包含：Web 应用、组件库、工具函数库、文档站点、后台管理系统。用 Monorepo 可以把这些放在同一个仓库里，共享配置和依赖。

**优点**：
- 代码共享方便，组件库修改后应用可以立即使用
- 统一的技术栈和构建配置
- 一次提交可以修改多个项目，保持版本一致性
- 方便做全量测试和代码审查

**缺点**：
- 仓库体积大，克隆时间长
- 权限管理复杂，不能细粒度控制不同项目的访问权限
- CI/CD 配置更复杂
- 对工具链要求高，需要用 Lerna、Nx、Turborepo、pnpm workspace 等工具管理

**常用工具**：
- **pnpm workspace**：轻量，用 pnpm 的 workspace 协议管理依赖
- **Turborepo**：Vercel 出品，支持远程缓存，构建速度快
- **Nx**：功能强大，适合大型 Monorepo
- **Lerna**：老牌工具，主要用于版本管理和发布

---

## 五、CI/CD 与部署

### 5.1 什么是 CI/CD？

**CI（Continuous Integration，持续集成）**：开发人员频繁地将代码合并到主干分支，每次合并都自动触发构建和测试，尽早发现问题。

**CD** 有两种含义：
- **Continuous Delivery（持续交付）**：代码通过测试后，自动部署到预发布环境，可以手动触发部署到生产环境
- **Continuous Deployment（持续部署）**：代码通过测试后，自动部署到生产环境，完全自动化

**常见的 CI/CD 工具**：
- **GitHub Actions**：GitHub 内置，配置简单，生态丰富
- **GitLab CI/CD**：GitLab 内置，功能强大
- **Jenkins**：老牌工具，插件丰富，适合自托管
- **CircleCI** / **Travis CI**：云端 CI/CD 服务

**典型的 CI/CD 流程**：
1. 开发者提交代码到 Git
2. 触发自动构建（安装依赖、编译打包）
3. 运行自动化测试（单元测试、E2E 测试）
4. 代码质量检查（ESLint、TypeScript 类型检查）
5. 构建产物部署到测试环境
6. 人工验收或自动验收后部署到生产环境

---

### 5.2 前端项目如何部署？

**静态托管**：
- **Vercel / Netlify**：专为前端设计，支持自动部署、预览部署、CDN 加速
- **GitHub Pages**：适合个人项目或文档站点
- **阿里云 OSS / 腾讯云 COS**：对象存储 + CDN，适合国内项目

**容器化部署**：
- 用 Docker 把前端应用打包成镜像
- 用 Nginx 作为静态文件服务器
- 部署到 Kubernetes 集群

**Nginx 配置示例**：

```nginx
server {
  listen 80;
  server_name example.com;
  root /var/www/html;
  index index.html;

  # 前端路由 history 模式支持
  location / {
    try_files $uri $uri/ /index.html;
  }

  # 静态资源缓存
  location ~* \.(js|css|png|jpg|jpeg|gif|ico|svg|woff|woff2)$ {
    expires 1y;
    add_header Cache-Control "public, immutable";
  }
}
```

---

### 5.3 什么是 Docker？前端项目为什么用 Docker？

Docker 是一种**容器化技术**，可以把应用和它的运行环境打包成一个独立的容器，保证在任何地方运行结果都一致。

**前端项目用 Docker 的好处**：

- **环境一致性**：开发环境、测试环境、生产环境完全一致，避免"在我电脑上能跑"的问题
- **部署标准化**：不管部署到什么服务器，都是运行同一个 Docker 镜像
- **隔离性**：不同项目可以用不同的 Node.js 版本，互不干扰
- **易于扩展**：配合 Kubernetes 可以快速水平扩展

**前端 Dockerfile 示例**：

```dockerfile
# 构建阶段
FROM node:18-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

# 运行阶段
FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
```

---

> **面试回答技巧**：工程化问题常结合实际项目经验，回答时可以说"我在上一家公司/项目中是怎么做的"。比如问到构建工具，可以讲你们项目为什么选 Webpack 或 Vite，遇到了什么问题怎么解决的。问到 CI/CD，可以讲你们的部署流程是怎样的，用了哪些工具。有实际经验会更有说服力。
