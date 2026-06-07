# CSS 面试题

> 前端面试中 CSS 相关的高频问题，涵盖 CSS 基础、布局、预处理器（SCSS/Less），以口语化回答为主，由浅入深排列。

---

## 一、基础概念

### 1.1 CSS 选择器有哪些？优先级怎么计算？

CSS 选择器有很多种，常用的有：

- **基础选择器**：标签选择器（`div`）、类选择器（`.class`）、ID 选择器（`#id`）、通配符（`*`）
- **组合选择器**：后代（`A B`）、子代（`A > B`）、相邻兄弟（`A + B`）、通用兄弟（`A ~ B`）
- **属性选择器**：`[type="text"]`、`[class^="btn"]`、`[href$=".pdf"]`
- **伪类选择器**：`:hover`、`:nth-child()`、`:not()`、`:focus`、`:checked`
- **伪元素选择器**：`::before`、`::after`、`::first-line`、`::placeholder`

**优先级计算规则**：选择器的优先级由四个部分组成，记作 `(a, b, c, d)`：

| 级别 | 对应选择器 | 数值 |
|------|-----------|------|
| a | 行内样式 `style=""` | 1 或 0 |
| b | ID 选择器 | 每个 +1 |
| c | 类、伪类、属性选择器 | 每个 +1 |
| d | 标签、伪元素选择器 | 每个 +1 |

比较时从左到右逐位比较，高位胜出就不再比较低位。比如 `(0, 1, 0, 0)` 大于 `(0, 0, 2, 0)`。

**`!important`** 可以覆盖任何优先级，但尽量少用，因为它会破坏 CSS 的可维护性。

---

### 1.2 盒模型是什么？标准盒模型和怪异盒模型的区别？

盒模型描述的是元素在页面中占据的空间，由四部分组成：内容（content）、内边距（padding）、边框（border）、外边距（margin）。

**标准盒模型**（`box-sizing: content-box`，默认值）：元素的 `width` 和 `height` 只包含内容区域，padding 和 border 会额外增加元素的总尺寸。

**怪异盒模型**（`box-sizing: border-box`）：元素的 `width` 和 `height` 包含内容 + padding + border，总尺寸就是设置的宽高值。

实际开发中几乎都会全局设置 `border-box`，因为更符合直觉：

```css
*, *::before, *::after {
  box-sizing: border-box;
}
```

---

### 1.3 有哪些常见的定位方式？

CSS 有五种定位方式：

**`static`**：默认值，元素按照正常文档流排列，不受 `top`、`left` 等属性影响。

**`relative`**：相对定位，元素相对于自身正常位置偏移，不脱离文档流，原来的位置仍然占据空间。

**`absolute`**：绝对定位，元素相对于最近的**已定位祖先元素**（非 `static`）定位，脱离文档流，不占据原来的空间。

**`fixed`**：固定定位，元素相对于浏览器视口定位，滚动时位置不变，常用于固定导航栏。

**`sticky`**：粘性定位，元素在滚动到指定阈值前表现为 `relative`，超过阈值后表现为 `fixed`。常用于表头吸顶、侧边栏跟随等效果。

---

### 1.4 `display` 属性有哪些常用值？

- **`block`**：块级元素，独占一行，可设置宽高
- **`inline`**：行内元素，不独占一行，不能设置宽高，宽高由内容决定
- **`inline-block`**：行内块元素，不独占一行，但可以设置宽高
- **`none`**：隐藏元素，不占据空间
- **`flex`**：弹性布局容器
- **`grid`**：网格布局容器
- **`table`** / **`table-cell`**：模拟表格布局

`display: none` 和 `visibility: hidden` 的区别：`none` 元素完全消失不占据空间，`hidden` 元素不可见但仍然占据原来的空间。

---

### 1.5 `float` 是什么？有什么副作用？如何清除浮动？

`float` 是 CSS 早期用来实现文字环绕图片效果的属性，后来也被用来做多列布局。元素设置 `float: left/right` 后会脱离正常文档流，向左或向右浮动。

**副作用**：浮动元素会导致父元素高度塌陷，因为浮动元素不再占据文档流中的空间，父元素计算高度时不会包含浮动子元素。

**清除浮动的方法**：

- **给父元素设置 `overflow: hidden` 或 `auto`**：触发 BFC，让父元素包含浮动子元素
- **使用伪元素 `::after` 清除浮动**（最常用）：

```css
.clearfix::after {
  content: '';
  display: block;
  clear: both;
}
```

现代布局中，`float` 已经基本被 `flex` 和 `grid` 取代，只在文字环绕等特殊场景使用。

---

## 二、布局方案

### 2.1 Flex 布局的核心概念？

Flex 布局是 CSS3 引入的一维布局方案，非常适合处理行或列方向的排列。

**核心概念**：

- **容器（Flex Container）**：设置 `display: flex` 的元素
- **项目（Flex Item）**：容器的直接子元素

**容器常用属性**：

- `flex-direction`：主轴方向，`row`（默认）/ `row-reverse` / `column` / `column-reverse`
- `justify-content`：主轴对齐方式，`flex-start` / `center` / `flex-end` / `space-between` / `space-around` / `space-evenly`
- `align-items`：交叉轴对齐方式，`stretch`（默认）/ `flex-start` / `center` / `flex-end` / `baseline`
- `flex-wrap`：是否换行，`nowrap`（默认）/ `wrap` / `wrap-reverse`
- `align-content`：多行时的整体对齐

**项目常用属性**：

- `flex: 1`：简写，等价于 `flex-grow: 1; flex-shrink: 1; flex-basis: 0%`，表示项目占据剩余空间的比例
- `flex-grow`：放大比例，默认 0
- `flex-shrink`：缩小比例，默认 1
- `flex-basis`：项目的基础大小
- `align-self`：单独设置项目的交叉轴对齐方式

---

### 2.2 Grid 布局的核心概念？

Grid 布局是 CSS 最强大的二维布局方案，可以同时处理行和列。

**核心概念**：

- **容器（Grid Container）**：设置 `display: grid` 的元素
- **项目（Grid Item）**：容器的直接子元素
- **网格线（Grid Line）**：划分网格的线
- **网格轨道（Grid Track）**：两个相邻网格线之间的空间（行或列）
- **网格单元（Grid Cell）**：最小的网格单位
- **网格区域（Grid Area）**：多个网格单元组成的区域

**容器常用属性**：

- `grid-template-columns` / `grid-template-rows`：定义列/行的大小，可以用 `fr` 单位（剩余空间的比例）、`repeat()` 函数
- `gap`：网格间距
- `justify-items` / `align-items`：单元格内项目的对齐
- `place-items`：`align-items` 和 `justify-items` 的简写

**项目常用属性**：

- `grid-column` / `grid-row`：指定项目占据的列/行范围
- `grid-area`：指定项目占据的区域名称或位置

```css
.container {
  display: grid;
  grid-template-columns: repeat(3, 1fr); /* 三列等宽 */
  gap: 16px;
}
```

---

### 2.3 如何实现水平垂直居中？

**Flex 方案**（最推荐）：

```css
.parent {
  display: flex;
  justify-content: center; /* 水平居中 */
  align-items: center;     /* 垂直居中 */
}
```

**Grid 方案**：

```css
.parent {
  display: grid;
  place-items: center; /* 水平和垂直同时居中 */
}
```

**绝对定位 + transform 方案**（兼容旧浏览器）：

```css
.parent { position: relative; }
.child {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
}
```

**绝对定位 + margin auto 方案**（需要子元素有固定宽高）：

```css
.child {
  position: absolute;
  top: 0; left: 0; right: 0; bottom: 0;
  margin: auto;
}
```

---

### 2.4 如何实现多列等高布局？

**Flex 方案**：

```css
.container {
  display: flex;
}
.item {
  flex: 1; /* 各列自动等高 */
}
```

**Grid 方案**：

```css
.container {
  display: grid;
  grid-auto-rows: 1fr; /* 行高自动等高 */
}
```

---

### 2.5 响应式布局有哪些实现方式？

**媒体查询（Media Query）**：

```css
@media (max-width: 768px) {
  /* 移动端样式 */
}
@media (min-width: 769px) and (max-width: 1024px) {
  /* 平板样式 */
}
@media (min-width: 1025px) {
  /* 桌面端样式 */
}
```

**弹性布局（Flex）**：利用 `flex-wrap` 和 `flex` 属性自动适应容器宽度。

**网格布局（Grid）**：利用 `auto-fit` 和 `minmax()` 实现自适应列数：

```css
.container {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
}
```

**百分比/视口单位**：用 `%`、`vw`、`vh` 等相对单位替代固定像素值。

**移动端适配方案**：
- **rem 方案**：根据根字体大小缩放，配合 `lib-flexible` 等库
- **vw/vh 方案**：直接用视口单位，1vw = 视口宽度的 1%
- **viewport 设置**：`<meta name="viewport" content="width=device-width, initial-scale=1.0">`

---

## 三、高级特性

### 3.1 什么是 BFC？如何触发？有什么作用？

BFC（Block Formatting Context，块级格式化上下文）是 CSS 中一个独立的渲染区域，里面的元素按照一定的规则进行布局，与外部元素互不影响。

**触发 BFC 的方式**：

- `float` 不为 `none`
- `position` 为 `absolute` 或 `fixed`
- `display` 为 `inline-block`、`table-cell`、`flex`、`grid` 等
- `overflow` 不为 `visible`

**BFC 的作用**：

- **清除浮动**：BFC 容器会包含内部的浮动元素，解决高度塌陷
- **防止 margin 重叠**：同一个 BFC 内的相邻块级元素 margin 会重叠，放在不同 BFC 中就不会
- **阻止元素被浮动元素覆盖**：BFC 元素不会与浮动元素重叠

---

### 3.2 什么是层叠上下文（Stacking Context）？

层叠上下文是 CSS 中元素在 Z 轴上的堆叠顺序。同一个层叠上下文中的元素按照一定规则决定谁在上谁在下。

**创建层叠上下文的方式**：

- `position` 不为 `static` 且 `z-index` 不为 `auto`
- `opacity` 小于 1
- `transform`、`filter`、`perspective` 不为 `none`
- `will-change` 指定了某些属性
- `isolation: isolate`

**层叠顺序规则**（从下到上）：

1. 背景和边框
2. 负 `z-index`
3. 块级元素
4. 浮动元素
5. 行内/行内块元素
6. `z-index: 0` / `auto`
7. 正 `z-index`

**注意**：`z-index` 只在已定位元素（非 `static`）上有效。两个元素的 `z-index` 比较只在同一个层叠上下文中有效，如果不在同一个层叠上下文中，需要比较它们所属层叠上下文的层级。

---

### 3.3 CSS 动画有哪些实现方式？

**Transition（过渡）**：用于简单的状态变化动画，比如 hover 效果。

```css
.box {
  transition: all 0.3s ease;
}
.box:hover {
  transform: scale(1.1);
}
```

**Animation（关键帧动画）**：用于复杂的、多阶段的动画。

```css
@keyframes slideIn {
  from { transform: translateX(-100%); opacity: 0; }
  to { transform: translateX(0); opacity: 1; }
}
.box {
  animation: slideIn 0.5s ease-out forwards;
}
```

**性能优化**：

- 优先使用 `transform` 和 `opacity` 做动画，这两个属性可以触发 GPU 加速
- 避免使用 `width`、`height`、`top`、`left` 等会触发重排的属性做动画
- 使用 `will-change` 提前告知浏览器哪些属性会变化

---

### 3.4 如何实现一个三角形？

利用 border 的特性实现：

```css
.triangle {
  width: 0;
  height: 0;
  border-left: 50px solid transparent;
  border-right: 50px solid transparent;
  border-bottom: 100px solid red;
}
```

原理是：当元素的宽高为 0 时，四个 border 会形成一个由四个三角形组成的正方形，把不需要的边设为 `transparent`，就只剩下一个三角形。

---

### 3.5 CSS 变量（Custom Properties）怎么用？

CSS 变量也叫自定义属性，以 `--` 开头定义，用 `var()` 函数引用。

```css
:root {
  --primary-color: #1890ff;
  --spacing-unit: 8px;
  --border-radius: 4px;
}

.btn {
  background: var(--primary-color);
  padding: calc(var(--spacing-unit) * 2);
  border-radius: var(--border-radius);
}

.btn:hover {
  --primary-color: #40a9ff; /* 局部覆盖 */
}
```

**优点**：
- 支持动态修改，通过 JS 改变变量值可以批量更新样式
- 支持作用域，可以在不同层级定义同名变量
- 比预处理器变量更灵活，因为预处理器变量在编译后就固定了

```javascript
// JS 动态修改 CSS 变量
document.documentElement.style.setProperty('--primary-color', '#ff4d4f');
```

---

## 四、预处理器

### 4.1 为什么要用 CSS 预处理器？

CSS 预处理器（如 SCSS/Sass、Less、Stylus）在原生 CSS 的基础上增加了变量、嵌套、混合、函数等编程特性，让 CSS 更易维护和复用。

**主要优势**：

- **变量**：统一管理颜色、间距等设计 token
- **嵌套**：让选择器层级关系更清晰，减少重复书写
- **混合（Mixin）**：复用样式片段，支持传参
- **继承（Extend）**：复用选择器的样式规则
- **函数/运算**：支持数学计算、颜色函数等
- **模块化**：通过 `@import` 或 `@use` 拆分文件

**注意**：预处理器最终都会编译成原生 CSS，浏览器不直接识别预处理器语法。

---

### 4.2 SCSS 和 Less 有什么区别？

| 特性 | SCSS/Sass | Less |
|------|-----------|------|
| 语法 | 两种语法：SCSS（类似 CSS）和 Sass（缩进式） | 类似 CSS |
| 变量符号 | `$color` | `@color` |
| 混合定义 | `@mixin` / `@include` | `.mixin()`（类选择器形式） |
| 继承 | `@extend` | `:extend()` |
| 条件语句 | `@if`、`@else`、`@for`、`@each`、`@while` | `when` 条件、`if()` 函数 |
| 循环 | 功能更强大 | 功能较简单 |
| 生态 | 更成熟，功能更丰富 | 较简单，易上手 |
| 编译工具 | node-sass、dart-sass | lessc |

**选择建议**：大型项目或需要复杂逻辑时用 SCSS；小型项目或团队偏好简单语法时用 Less。

---

### 4.3 SCSS 常用特性有哪些？

**变量**：

```scss
$primary: #1890ff;
$spacing: 8px;

.btn {
  color: $primary;
  padding: $spacing * 2;
}
```

**嵌套**：

```scss
.card {
  padding: 16px;
  
  &:hover { box-shadow: 0 4px 12px rgba(0,0,0,0.1); }
  
  &__title { font-size: 18px; }
  &__body { margin-top: 8px; }
}
// 编译后：.card { ... } .card:hover { ... } .card__title { ... }
```

**混合（Mixin）**：

```scss
@mixin flex-center {
  display: flex;
  justify-content: center;
  align-items: center;
}

@mixin text-ellipsis($lines: 1) {
  @if $lines == 1 {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  } @else {
    display: -webkit-box;
    -webkit-line-clamp: $lines;
    -webkit-box-orient: vertical;
    overflow: hidden;
  }
}

.card {
  @include flex-center;
  .title { @include text-ellipsis(2); }
}
```

**继承（Extend）**：

```scss
%btn-base {
  display: inline-block;
  padding: 8px 16px;
  border-radius: 4px;
}

.btn-primary {
  @extend %btn-base;
  background: $primary;
  color: white;
}

.btn-secondary {
  @extend %btn-base;
  background: white;
  border: 1px solid $primary;
}
```

**函数**：

```scss
@function rem($px) {
  @return $px / 16px * 1rem;
}

.title { font-size: rem(24px); }
```

**模块化**：

```scss
// _variables.scss
$primary: #1890ff;

// main.scss
@use 'variables' as v;
.btn { color: v.$primary; }

// 或 @import（已逐渐被 @use 取代）
@import 'variables';
```

---

### 4.4 Less 常用特性有哪些？

**变量**：

```less
@primary: #1890ff;
@spacing: 8px;

.btn {
  color: @primary;
  padding: @spacing * 2;
}
```

**嵌套**：

```less
.card {
  padding: 16px;
  
  &:hover { box-shadow: 0 4px 12px rgba(0,0,0,0.1); }
  
  &-title { font-size: 18px; }
  &-body { margin-top: 8px; }
}
// 编译后：.card { ... } .card:hover { ... } .card-title { ... }
```

**混合（Mixin）**：

```less
.flex-center() {
  display: flex;
  justify-content: center;
  align-items: center;
}

.text-ellipsis(@lines: 1) when (@lines = 1) {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.text-ellipsis(@lines: 1) when (@lines > 1) {
  display: -webkit-box;
  -webkit-line-clamp: @lines;
  -webkit-box-orient: vertical;
  overflow: hidden;
}

.card {
  .flex-center();
  .title { .text-ellipsis(2); }
}
```

**运算**：

```less
@base: 8px;
@double: @base * 2;  // 16px
@half: @base / 2;    // 4px
```

**导入**：

```less
@import "variables";
@import "mixins";
```

---

### 4.5 预处理器变量和 CSS 变量有什么区别？

| 特性 | 预处理器变量（SCSS/Less） | CSS 变量（Custom Properties） |
|------|------------------------|------------------------------|
| 编译时机 | 编译时确定，编译后变成固定值 | 运行时生效，可以动态修改 |
| 作用域 | 文件级/块级作用域 | DOM 级作用域，可继承和覆盖 |
| JS 修改 | 不能直接修改 | 可以通过 `setProperty` 动态修改 |
| 性能 | 编译后无运行时开销 | 运行时计算，略有开销 |
| 浏览器支持 | 不依赖浏览器，编译后就是普通 CSS | IE 不支持，现代浏览器都支持 |
| 适用场景 | 设计 token、静态值 | 主题切换、动态样式、组件库 |

**实际项目中的做法**：用预处理器变量管理设计系统的基础 token（颜色、间距、字体等），编译成 CSS 变量，这样既保留了预处理器的组织能力，又能享受 CSS 变量的动态性。

```scss
// 预处理器管理设计 token
$colors: (
  primary: #1890ff,
  success: #52c41a,
  danger: #ff4d4f,
);

// 编译成 CSS 变量
:root {
  @each $name, $value in $colors {
    --color-#{$name}: #{$value};
  }
}
// 编译结果：:root { --color-primary: #1890ff; --color-success: #52c41a; ... }
```

---

## 五、工程化与最佳实践

### 5.1 CSS 模块化方案有哪些？

**BEM 命名规范**：

BEM 是 Block（块）、Element（元素）、Modifier（修饰符）的缩写，通过命名约定实现样式隔离。

```css
.block { }
.block__element { }      /* 元素用 __ 连接 */
.block--modifier { }     /* 修饰符用 -- 连接 */

/* 示例 */
.card { }
.card__title { }
.card__body { }
.card--large { }
.card--primary { }
```

**CSS Modules**：

通过构建工具（如 Webpack）将类名编译成唯一的哈希值，实现局部作用域。

```css
/* styles.module.css */
.title { color: red; }
```

```javascript
import styles from './styles.module.css';
// styles.title 编译后变成类似 "title_a3f7b2"
```

**CSS-in-JS**：

在 JS 中写 CSS，如 Styled-components、Emotion，通过 JS 运行时生成唯一类名。

**Tailwind CSS**：

原子化 CSS 方案，通过工具类直接组合样式，不需要写自定义 CSS。

```html
<div class="flex items-center justify-between p-4 bg-white rounded-lg shadow">
```

---

### 5.2 如何处理浏览器兼容性问题？

**CSS 前缀**：使用 Autoprefixer 自动添加 `-webkit-`、`-moz-` 等浏览器前缀。

**Polyfill**：对不支持的 CSS 特性使用 JS 库做兼容，比如 `css-vars-ponyfill` 让 IE 支持 CSS 变量。

**渐进增强 / 优雅降级**：
- 渐进增强：先保证基础功能在所有浏览器可用，再在支持的浏览器上增强
- 优雅降级：先实现完整功能，再在不支持的浏览器上做降级处理

**@supports 条件查询**：

```css
@supports (display: grid) {
  .container { display: grid; }
}
@supports not (display: grid) {
  .container { display: flex; }
}
```

**PostCSS**：用 PostCSS 插件处理兼容性问题，如 `postcss-preset-env` 自动将现代 CSS 转译为兼容代码。

---

### 5.3 如何组织大型项目的 CSS 结构？

推荐按功能或层级拆分文件，常见的组织方式：

```
styles/
├── base/           # 基础样式
│   ├── reset.css   # 样式重置
│   ├── variables   # CSS 变量/设计 token
│   └── typography  # 字体排版
├── components/     # 组件样式
│   ├── button.scss
│   ├── card.scss
│   └── form.scss
├── layouts/        # 布局样式
│   ├── header.scss
│   ├── sidebar.scss
│   └── footer.scss
├── pages/          # 页面级样式
│   ├── home.scss
│   └── about.scss
├── utils/          # 工具类
│   ├── mixins.scss
│   └── functions.scss
└── main.scss       # 入口文件，统一导入
```

**入口文件示例**：

```scss
// main.scss
@use 'base/reset';
@use 'base/variables';
@use 'base/typography';
@use 'components/button';
@use 'components/card';
@use 'layouts/header';
@use 'layouts/sidebar';
```

**原则**：
- 一个文件只做一件事
- 组件样式独立，避免全局污染
- 使用 `@use` 替代 `@import`，避免命名冲突
- 公共变量和混合放在 base/utils 中统一管理

---

> **面试回答技巧**：CSS 面试题常结合实际场景，回答时可以结合你项目中的具体做法。比如问到布局，可以说"我在项目中主要用 Flex 做一维布局、Grid 做二维布局，对于旧浏览器用绝对定位做兜底"。问到预处理器，可以讲你们团队选 SCSS 的原因和实际使用经验。
