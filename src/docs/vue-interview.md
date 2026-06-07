# Vue 面试题

> Vue 相关的高频面试题，涵盖 Vue 2 和 Vue 3，以口语化回答为主，由浅入深排列。

---

## 一、基础概念

### 1.1 Vue 的响应式原理是什么？

Vue 的响应式就是**数据变化时视图自动更新**，核心是数据劫持和依赖收集。

**Vue 2** 用 `Object.defineProperty` 实现响应式。在初始化时递归遍历 data 中的每个属性，用 `Object.defineProperty` 把它们转成 getter/setter。getter 中做依赖收集，记录哪些 Watcher 依赖了这个属性；setter 中做派发更新，通知所有依赖这个属性的 Watcher 重新渲染。

Vue 2 有几个限制：一是无法检测对象属性的添加和删除，需要用 `Vue.set` / `Vue.delete`；二是无法检测数组索引的直接赋值和长度修改，Vue 重写了数组的 `push`、`pop`、`splice` 等方法来解决这个问题。

**Vue 3** 改用 `Proxy` 实现响应式。`Proxy` 可以直接代理整个对象，不需要递归遍历每个属性。它能检测属性的添加和删除，也能检测数组的变化，从根本上解决了 Vue 2 的限制。

```javascript
// Vue 2：Object.defineProperty
Object.defineProperty(obj, 'name', {
  get() { /* 收集依赖 */ },
  set(newVal) { /* 派发更新 */ },
});

// Vue 3：Proxy
const proxy = new Proxy(obj, {
  get(target, key) { /* 收集依赖 */ },
  set(target, key, value) { /* 派发更新 */ },
});
```

---

### 1.2 Vue 2 和 Vue 3 有哪些主要区别？

| 维度 | Vue 2 | Vue 3 |
|------|-------|-------|
| 响应式 | `Object.defineProperty` | `Proxy` |
| API 风格 | Options API | Composition API + Options API |
| 组件逻辑复用 | Mixin（容易命名冲突） | Composable（组合式函数） |
| 根节点 | 单根节点 | 支持多根节点（Fragment） |
| 生命周期 | `beforeCreate`、`created` 等 | `setup` 替代 `beforeCreate`/`created`，其余加 `on` 前缀 |
| v-model | 一个组件只能一个 v-model | 支持多个 v-model，可自定义参数名 |
| 内置组件 | `Transition` | `Transition`、`TransitionGroup`、`Teleport`、`Suspense` |
| TypeScript | 支持较弱 | 原生支持，类型推导更好 |
| 性能 | 虚拟 DOM 重写，编译时优化更多 | 静态提升、Patch Flag、Tree Shaking 更彻底 |
| 包体积 | 较大 | 支持按需引入，体积更小 |

**选择建议**：新项目直接用 Vue 3，老项目如果没有特殊需求可以继续用 Vue 2，但建议逐步迁移。

---

### 1.3 Vue 的生命周期有哪些？分别在什么时候调用？

**Vue 2 生命周期**：

- `beforeCreate`：实例刚创建，data 和 methods 都还没有初始化
- `created`：data 和 methods 已初始化，可以访问数据，但 DOM 还没挂载，适合发初始化请求
- `beforeMount`：模板编译完成，准备挂载到 DOM
- `mounted`：DOM 挂载完成，可以操作 DOM 元素，适合初始化需要 DOM 的第三方库
- `beforeUpdate`：数据变化后，DOM 更新之前
- `updated`：DOM 更新完成
- `beforeDestroy`：实例销毁之前，适合清除定时器、解绑事件
- `destroyed`：实例销毁完成

**Vue 3 生命周期**（Composition API）：

- `setup()`：替代 `beforeCreate` 和 `created`，在实例创建前执行
- `onBeforeMount`：对应 `beforeMount`
- `onMounted`：对应 `mounted`
- `onBeforeUpdate`：对应 `beforeUpdate`
- `onUpdated`：对应 `updated`
- `onBeforeUnmount`：对应 `beforeDestroy`
- `onUnmounted`：对应 `destroyed`
- `onActivated` / `onDeactivated`：`keep-alive` 组件激活/停用时触发

**实际开发中最常用的**：`created`（或 `setup`）发请求初始化数据、`mounted` 操作 DOM、`beforeDestroy`（或 `onBeforeUnmount`）清理副作用。

---

### 1.4 computed 和 watch 的区别？

`computed` 是**计算属性**，基于已有的响应式数据派生出一个新值，有缓存机制。只有当它依赖的数据发生变化时才会重新计算，否则直接返回缓存结果。适合有返回值的派生数据，比如过滤后的列表、格式化后的文本。

`watch` 是**侦听器**，监听某个数据的变化，执行副作用操作。没有返回值，适合在数据变化时执行异步操作或开销较大的逻辑，比如监听搜索词变化后发请求、监听路由变化后重新加载数据。

**选择原则**：能用 `computed` 就用 `computed`，需要执行副作用（发请求、操作 DOM）时才用 `watch`。

```javascript
// computed：有缓存，依赖不变就不重新计算
const fullName = computed(() => firstName.value + ' ' + lastName.value);

// watch：监听变化，执行副作用
watch(searchKeyword, (newVal) => {
  fetchSearchResults(newVal);
}, { immediate: true, deep: true });
```

---

### 1.5 v-if 和 v-show 的区别？

`v-if` 是**真正的条件渲染**，条件为 false 时元素不会被渲染到 DOM 中，切换时会销毁和重建元素及其内部的事件监听器和子组件。

`v-show` 是**CSS 级别的切换**，不管条件如何，元素都会被渲染到 DOM 中，只是通过 `display: none` 来控制显示隐藏。

**选择原则**：
- 需要频繁切换显示隐藏的用 `v-show`，因为切换开销小
- 运行时条件很少变化、或者初始条件为 false 时不需要渲染的用 `v-if`，因为初始渲染开销更小
- `v-if` 可以和 `v-else`、`v-else-if` 配合使用，`v-show` 不行

---

## 二、组件通信

### 2.1 Vue 组件通信有哪些方式？

**父子组件通信**：

- **Props / Emit**：父组件通过 props 向子组件传数据，子组件通过 `$emit`（Vue 2）或 `defineEmits`（Vue 3）向父组件发事件
- **`$refs`**：父组件通过 ref 直接访问子组件实例，调用子组件方法或读取数据

**子父组件通信**：

- **`$emit` / `defineEmits`**：子组件触发自定义事件，父组件监听并处理

**兄弟组件通信**：

- **事件总线（Event Bus）**：Vue 2 中常用，创建一个空的 Vue 实例作为事件中心；Vue 3 推荐用 `mitt` 库替代
- **Vuex / Pinia**：通过状态管理，任何组件都可以读写共享状态

**跨层级通信**：

- **Provide / Inject**：祖先组件 provide 数据，后代组件 inject 使用，Vue 3 中支持响应式
- **Vuex / Pinia**：全局状态管理

**Vue 3 新增方式**：

- **`defineModel`**：简化 v-model 的双向绑定
- **`useAttrs` / `useSlots`**：访问透传的属性和插槽

```javascript
// Vue 3：Provide / Inject（响应式）
// 父组件
const theme = ref('dark');
provide('theme', theme);

// 子孙组件
const theme = inject('theme');
```

---

### 2.2 Vuex 和 Pinia 有什么区别？

| 特性 | Vuex | Pinia |
|------|------|-------|
| Vue 版本 | Vue 2 / Vue 3（Vuex 4） | 仅 Vue 3 |
| API 风格 | 选项式（state/mutations/actions/getters） | 组合式，更接近 Composition API |
| Mutations | 必须通过 mutation 修改 state | 直接修改，没有 mutation |
| TypeScript | 类型支持较弱 | 完整的 TypeScript 支持 |
| 模块化 | 需要嵌套 modules | 每个 store 独立，天然模块化 |
| 体积 | 较大 | 更小（约 1KB） |
| DevTools | 支持 | 支持 |
| 官方推荐 | Vue 2 推荐 | Vue 3 官方推荐，已取代 Vuex |

**选择建议**：Vue 3 新项目直接用 Pinia，更简洁、类型更好、学习成本更低。Vue 2 项目用 Vuex。

```javascript
// Pinia 示例
export const useCounterStore = defineStore('counter', () => {
  const count = ref(0);
  const doubleCount = computed(() => count.value * 2);
  function increment() { count.value++; }
  return { count, doubleCount, increment };
});
```

---

## 三、指令与模板

### 3.1 Vue 常用指令有哪些？

- **`v-bind`（缩写 `:`）**：动态绑定属性，如 `:class`、`:style`、`:src`
- **`v-on`（缩写 `@`）**：绑定事件，如 `@click`、`@input`
- **`v-model`**：表单双向绑定
- **`v-if` / `v-else-if` / `v-else`**：条件渲染
- **`v-show`**：显示隐藏（CSS 切换）
- **`v-for`**：列表渲染，需要配合 `:key` 使用
- **`v-slot`（缩写 `#`）**：插槽
- **`v-once`**：只渲染一次，后续数据变化不更新
- **`v-html`**：渲染 HTML 字符串（注意 XSS 风险）
- **`v-text`**：渲染纯文本
- **`v-pre`**：跳过编译，直接输出原始模板

---

### 3.2 v-for 为什么要加 key？

`key` 是 Vue 用来识别节点身份的标识，帮助 Vue 在 diff 算法中高效地更新虚拟 DOM。

当列表数据变化时，Vue 会对比新旧虚拟 DOM 节点。如果没有 `key`，Vue 默认使用"就地复用"策略，按顺序对比，可能导致不必要的 DOM 操作。比如在列表头部插入一个元素，没有 key 的话所有元素都会被更新。

有了 `key`，Vue 可以准确找到哪些节点是新增的、删除的、移动的，只做最小必要的 DOM 操作，提升渲染性能。

**key 的选择**：使用数据的唯一标识（如 `id`），不要用 `index`，因为当列表顺序变化时 index 也会变化，失去 key 的意义。

---

### 3.3 Vue 的 diff 算法是怎么工作的？

Vue 的 diff 算法采用的是**同层比较、双向遍历**的策略。

核心思路是：只比较同一层级的节点，不跨层级比较。比较时从新旧节点的头尾同时开始，向中间靠拢。

具体步骤：

1. **头对头比较**：旧头和新头是否是同一个节点（key 和标签相同）
2. **尾对尾比较**：旧尾和新尾是否是同一个节点
3. **头对尾比较**：旧头和新尾是否是同一个节点（处理反转）
4. **尾对头比较**：旧尾和新头是否是同一个节点（处理移动）
5. **以上都不匹配**：用 key 建立旧节点到新节点的映射，找到可复用的节点

Vue 3 的 diff 算法在此基础上做了优化，引入了**静态提升**（不参与 diff 的静态节点在编译时提升）、**Patch Flag**（标记动态节点需要 diff 的具体属性）、**Tree Shaking**（编译时优化）等，性能更好。

---

## 四、Vue Router

### 4.1 Vue Router 的两种模式有什么区别？

**Hash 模式**：URL 中带 `#`，如 `http://example.com/#/home`。利用 `hashchange` 事件监听路由变化。优点是兼容性好，不需要服务端配置。缺点是 URL 不美观，SEO 不友好。

**History 模式**：URL 不带 `#`，如 `http://example.com/home`。利用 HTML5 的 `history.pushState` / `replaceState` API。优点是 URL 更美观，对 SEO 更友好。缺点是刷新页面会向服务端发请求，需要服务端配置把所有路由都指向 `index.html`（即 fallback）。

**选择建议**：需要 SEO 或对 URL 美观有要求的用 History 模式，其他情况两种都可以。当前项目用的就是 Hash 模式。

```javascript
// Vue 3：创建路由
const router = createRouter({
  history: createWebHashHistory(), // Hash 模式
  // history: createWebHistory(),  // History 模式
  routes: [
    { path: '/', component: Home },
    { path: '/about', component: About },
  ],
});
```

---

### 4.2 路由守卫有哪些？

**全局守卫**：
- `router.beforeEach`：全局前置守卫，每次路由跳转前触发，常用于权限校验
- `router.afterEach`：全局后置守卫，路由跳转后触发，常用于页面统计、修改标题

**路由独享守卫**：
- `beforeEnter`：在路由配置中定义，只对该路由生效

**组件内守卫**：
- `onBeforeRouteEnter`：进入组件前（Vue 3 Composition API）
- `onBeforeRouteUpdate`：路由参数变化时（Vue 3 Composition API）
- `onBeforeRouteLeave`：离开组件前，常用于阻止用户在未保存时离开

```javascript
// 全局前置守卫：权限校验
router.beforeEach((to, from, next) => {
  if (to.meta.requiresAuth && !isLoggedIn()) {
    next('/login');
  } else {
    next();
  }
});
```

---

## 五、状态管理与数据流

### 5.1 什么是单向数据流？

Vue 的数据流是**单向的**：父组件通过 props 向子组件传递数据，子组件不能直接修改 props，只能通过触发事件（emit）通知父组件修改。

这样做的好处是：数据流向清晰，容易追踪数据变化；避免子组件意外修改父组件状态，降低 bug 风险。

如果子组件需要修改父组件的数据，正确做法是：
1. 子组件 `$emit` 一个事件
2. 父组件监听事件并修改数据
3. 数据变化后通过 props 重新传给子组件

如果需要"双向绑定"的效果，Vue 2 中可以用 `.sync` 修饰符，Vue 3 中可以用 `v-model` 配合 `defineModel`。

---

### 5.2 什么是 Composable（组合式函数）？

Composable 是 Vue 3 中用来**复用有状态逻辑**的方式，本质上是一个封装了响应式逻辑的函数，以 `use` 开头命名。

它解决了 Vue 2 中 Mixin 的痛点：Mixin 容易命名冲突、数据来源不清晰、难以追溯。Composable 通过函数参数和返回值的方式，让数据来源和逻辑关系非常清晰。

```javascript
// useMousePosition.js
export function useMousePosition() {
  const x = ref(0);
  const y = ref(0);

  const update = (event) => {
    x.value = event.pageX;
    y.value = event.pageY;
  };

  onMounted(() => window.addEventListener('mousemove', update));
  onUnmounted(() => window.removeEventListener('mousemove', update));

  return { x, y };
}

// 在组件中使用
const { x, y } = useMousePosition();
```

**常用 Composable**：`useFetch`（数据请求）、`useLocalStorage`（本地存储）、`useDebounce`（防抖）、`useEventListener`（事件监听）等。

---

## 六、性能优化

### 6.1 Vue 项目有哪些性能优化手段？

**代码层面**：
- 使用 `v-if` 替代 `v-show` 减少初始渲染开销（不频繁切换的场景）
- 使用 `computed` 缓存计算结果，避免重复计算
- 使用 `v-once` 让静态内容只渲染一次
- 合理使用 `key`，避免不必要的 DOM 更新
- 大列表使用虚拟滚动（`vue-virtual-scroller`）

**组件层面**：
- 异步组件（`defineAsyncComponent`）按需加载
- `keep-alive` 缓存组件状态，避免重复渲染
- 合理拆分组件，避免一个组件过大

**状态管理**：
- Pinia/Vuex 中按模块拆分 store
- 避免在 store 中存储大量非必要数据

**构建层面**：
- 路由懒加载
- 第三方库按需引入（如 Element Plus 的按需引入）
- 开启 Gzip 压缩
- 使用 `unplugin-vue-components` 自动注册组件

```javascript
// Vue 3：异步组件
const HeavyComponent = defineAsyncComponent(() =>
  import('./HeavyComponent.vue')
);

// 路由懒加载
const routes = [
  { path: '/about', component: () => import('./views/About.vue') },
];
```

---

### 6.2 keep-alive 的作用和使用方式？

`keep-alive` 是 Vue 的内置组件，用于缓存组件实例，避免组件在切换时被销毁和重建，从而保留组件的状态和 DOM。

**使用方式**：

```html
<!-- 缓存所有子组件 -->
<keep-alive>
  <component :is="currentComponent" />
</keep-alive>

<!-- 只缓存 name 为 A 或 B 的组件 -->
<keep-alive include="A,B">
  <component :is="currentComponent" />
</keep-alive>

<!-- 排除 name 为 C 的组件 -->
<keep-alive exclude="C">
  <component :is="currentComponent" />
</keep-alive>
```

**生命周期**：被 `keep-alive` 缓存的组件在激活和停用时会触发 `onActivated` 和 `onDeactivated`（Vue 3）。

**适用场景**：标签页切换、列表页和详情页之间来回切换、表单填写中途切换页面后保留输入内容。

---

### 6.3 什么是虚拟 DOM？Vue 为什么用虚拟 DOM？

虚拟 DOM 就是用 JavaScript 对象来描述真实 DOM 结构。当数据变化时，Vue 先创建新的虚拟 DOM 树，然后和旧的虚拟 DOM 树做 diff 比较，计算出最小的更新操作，最后批量更新真实 DOM。

**为什么用虚拟 DOM**：

1. **减少直接操作 DOM**：直接操作 DOM 性能开销大，虚拟 DOM 把多次 DOM 操作合并成一次批量更新
2. **跨平台**：虚拟 DOM 不依赖浏览器环境，可以渲染到不同目标（如服务端渲染 SSR、小程序）
3. **开发体验**：开发者只需要关心数据变化，不需要手动操作 DOM

**Vue 3 的优化**：Vue 3 在编译阶段做了更多优化，比如静态提升（静态节点不参与 diff）、Patch Flag（标记动态属性，只 diff 变化的部分）、Block Tree（追踪动态子节点），让 diff 更高效。

---

## 七、进阶问题

### 7.1 Vue 3 的 Composition API 相比 Options API 有什么优势？

**逻辑组织更灵活**：Options API 按选项分类（data、methods、computed、watch），同一个功能的代码分散在不同选项中。Composition API 可以把同一个功能的代码组织在一起，更符合逻辑思维。

**逻辑复用更方便**：Options API 用 Mixin 复用逻辑，容易命名冲突、来源不清。Composition API 用 Composable 函数复用，清晰明确。

**TypeScript 支持更好**：Composition API 基于函数和响应式引用，类型推导更自然。

**Tree Shaking 更友好**：Composition API 的 API 都是按需导入的，没用到的功能不会打包。

**不过 Options API 也有优势**：对新手更友好，上手简单，适合简单组件。实际项目中两者可以混用。

---

### 7.2 什么是 nextTick？为什么需要它？

`nextTick` 是 Vue 提供的一个方法，作用是**在下一次 DOM 更新循环结束后执行回调**。

Vue 的数据更新是异步的。当数据变化后，DOM 不会立即更新，而是在下一个"tick"中批量更新。所以如果在数据变化后立即操作 DOM，拿到的还是旧的 DOM。

```javascript
const message = ref('old');
message.value = 'new';

// 这时候 DOM 还没更新
console.log(document.querySelector('.msg').textContent); // 'old'

// 需要用 nextTick 等待 DOM 更新
nextTick(() => {
  console.log(document.querySelector('.msg').textContent); // 'new'
});
```

**常见使用场景**：在修改数据后立即操作更新后的 DOM、在 created 中需要访问 DOM（虽然一般不建议）、在自定义指令中等待 DOM 更新。

---

### 7.3 Vue 的 SSR（服务端渲染）是什么？有什么优缺点？

SSR 就是在**服务端把 Vue 组件渲染成 HTML 字符串**，直接返回给浏览器，而不是在浏览器中渲染。

**优点**：
- **SEO 友好**：搜索引擎爬虫可以直接抓取到完整的 HTML 内容
- **首屏加载快**：用户不需要等待 JS 加载和执行就能看到页面内容
- **社交分享友好**：分享链接时可以正确显示页面标题和描述

**缺点**：
- **开发复杂度高**：需要处理服务端和客户端的差异（如 `window`、`document` 只在客户端可用）
- **服务器负载大**：每次请求都要在服务端渲染页面
- **构建和部署更复杂**：需要 Node.js 服务器环境

**Vue 3 的 SSR 方案**：Nuxt 3 是 Vue 3 官方推荐的全栈框架，内置 SSR 支持。也可以用 Vite 的 SSR 模式手动搭建。

---

### 7.4 Vue 项目如何做权限控制？

**路由级权限**：在路由守卫中检查用户角色和权限，无权限则跳转到登录页或 403 页面。

```javascript
router.beforeEach((to, from, next) => {
  const token = localStorage.getItem('token');
  if (to.meta.requiresAuth && !token) {
    next({ path: '/login', query: { redirect: to.fullPath } });
  } else {
    next();
  }
});
```

**按钮级权限**：自定义指令 `v-permission`，根据权限控制按钮的显示隐藏。

```javascript
// 自定义指令
app.directive('permission', {
  mounted(el, binding) {
    const permissions = getUserPermissions();
    if (!permissions.includes(binding.value)) {
      el.parentNode?.removeChild(el);
    }
  },
});

// 使用
<button v-permission="'admin:delete'">删除</button>
```

**接口级权限**：在请求拦截器中统一添加 Token，在响应拦截器中处理 401/403 等权限错误。

---

> **面试回答技巧**：Vue 面试常问原理和对比类问题。回答时先说结论，再展开细节，最后结合项目经验。比如问到响应式原理，可以先说 Vue 2 用 `defineProperty`、Vue 3 用 `Proxy`，再说各自的特点和限制，最后讲你在项目中遇到的响应式相关问题和解决方式。
