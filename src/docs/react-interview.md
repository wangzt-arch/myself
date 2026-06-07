# React 面试题

> React 相关的高频面试题，涵盖核心概念、Hooks、状态管理、性能优化，以口语化回答为主，由浅入深排列。

---

## 一、核心概念

### 1.1 React 的核心思想是什么？

React 的核心思想可以概括为三点：

**组件化**：把页面拆分成一个个独立的、可复用的组件，每个组件管理自己的状态和渲染逻辑。就像搭积木一样，用小组件组合成大页面。

**声明式编程**：开发者只需要描述 UI 应该长什么样（状态 → 视图的映射），不需要关心怎么去操作 DOM 更新视图。React 会自动帮你处理。

**单向数据流**：数据从父组件通过 props 流向子组件，子组件不能直接修改父组件的数据。如果需要修改，通过回调函数通知父组件。

React 的哲学是 **UI = f(state)**，视图是状态的函数，状态变了视图就自动更新。

---

### 1.2 虚拟 DOM 和 diff 算法是怎么工作的？

**虚拟 DOM** 就是用 JavaScript 对象来描述真实 DOM 结构。当状态变化时，React 会创建新的虚拟 DOM 树，然后和旧的做对比，计算出最小的更新操作，最后批量更新真实 DOM。

**diff 算法的核心策略**：

1. **同层比较**：只比较同一层级的节点，不跨层级比较。这把 O(n³) 的时间复杂度降到了 O(n)。
2. **key 匹配**：通过 `key` 来识别节点。列表渲染时必须加 `key`，帮助 React 找到对应关系。
3. **类型判断**：不同类型的元素（如 `div` 变成 `span`）会直接销毁重建，不会尝试复用。

**为什么不能用 index 做 key**：当列表发生插入、删除、排序操作时，index 会变化，导致 React 错误地复用节点，产生不必要的更新甚至状态错乱。应该用数据的唯一标识（如 `id`）。

---

### 1.3 React 和 Vue 有什么区别？

| 维度 | React | Vue |
|------|-------|-----|
| 设计理念 | UI = f(state)，函数式思想 | 响应式数据驱动，渐进式框架 |
| 数据更新 | 手动 setState（类组件）/ useState（函数组件） | 自动响应式，修改数据即可 |
| 模板语法 | JSX（JavaScript + XML） | 模板语法 + JSX（Vue 3） |
| 组件写法 | 函数组件 + Hooks 为主 | Options API + Composition API |
| 状态管理 | Redux / Zustand / Jotai | Pinia / Vuex |
| 学习曲线 | 相对陡峭，需要理解函数式编程 | 相对平缓，上手快 |
| 生态 | 非常庞大，React Native 跨端 | 生态丰富，官方维护 UI 组件库 |
| TypeScript | 支持良好 | Vue 3 原生支持更好 |
| 背后团队 | Meta（Facebook） | 尤雨溪及社区 |

**选择建议**：没有绝对的好坏，看团队技术栈和项目需求。React 生态更大、大公司用得多；Vue 上手快、中文文档友好。

---

### 1.4 类组件和函数组件的区别？

**类组件**：用 `class` 关键字定义，通过 `this.state` 管理状态，通过 `this.setState` 更新状态，有生命周期方法。

**函数组件**：用函数定义，早期只能做纯展示组件（无状态），React 16.8 引入 Hooks 后，函数组件可以完全替代类组件。

| 特性 | 类组件 | 函数组件 + Hooks |
|------|--------|-----------------|
| 状态管理 | `this.state` / `this.setState` | `useState` / `useReducer` |
| 副作用处理 | 生命周期方法 | `useEffect` |
| 逻辑复用 | HOC / Render Props | 自定义 Hooks |
| 代码量 | 较多（this 绑定、生命周期） | 更简洁 |
| 性能 | 需手动 `shouldComponentUpdate` | `React.memo` 优化 |
| 趋势 | 逐渐被淘汰 | 官方推荐 |

**现在推荐全部用函数组件 + Hooks**，代码更简洁、逻辑复用更方便、更容易测试。

---

## 二、Hooks

### 2.1 useState 的使用注意事项？

`useState` 是最基础的 Hook，用于在函数组件中管理状态。

```javascript
const [count, setCount] = useState(0);
```

**注意事项**：

1. **setState 是异步的**：在 React 事件处理中，多次调用 setState 会被合并成一次更新。如果新状态依赖旧状态，要用函数式更新。

```javascript
// ❌ 可能不正确（依赖旧值）
setCount(count + 1);
setCount(count + 1); // 还是只加 1

// ✅ 正确（函数式更新）
setCount(prev => prev + 1);
setCount(prev => prev + 1); // 加 2
```

2. **初始值只生效一次**：useState 的参数只在组件首次渲染时使用，后续重新渲染时忽略。如果需要根据 props 计算初始值，可以传函数。

```javascript
const [data, setData] = useState(() => expensiveCompute(props.id));
```

3. **状态不可变**：不要直接修改状态对象，要创建新对象。

```javascript
// ❌ 直接修改
user.name = 'Tom';
setUser(user);

// ✅ 创建新对象
setUser({ ...user, name: 'Tom' });
```

---

### 2.2 useEffect 的使用场景和注意事项？

`useEffect` 用于处理副作用，比如请求数据、操作 DOM、订阅事件等。

**基本用法**：

```javascript
// 每次渲染后执行
useEffect(() => { /* ... */ });

// 只在首次渲染后执行（空依赖数组）
useEffect(() => { /* ... */ }, []);

// 依赖项变化时执行
useEffect(() => { /* ... */ }, [dep1, dep2]);
```

**注意事项**：

1. **依赖数组要完整**：useEffect 内部用到的响应式变量都应该放在依赖数组中，否则可能拿到旧值。可以用 `eslint-plugin-react-hooks` 的 `exhaustive-deps` 规则自动检查。

2. **清理副作用**：如果 useEffect 中有订阅、定时器等，需要返回清理函数，否则会造成内存泄漏。

```javascript
useEffect(() => {
  const timer = setInterval(() => { /* ... */ }, 1000);
  return () => clearInterval(timer); // 清理
}, []);
```

3. **不要把 useEffect 当 componentDidMount / componentDidUpdate / componentWillUnmount 的简单替代**：理解它的本质是"渲染后执行"，根据实际需求选择是否加依赖。

---

### 2.3 useRef 的作用是什么？

`useRef` 有两个主要用途：

**引用 DOM 元素**：获取 DOM 节点的引用，用于聚焦、测量尺寸、操作 DOM 等。

```javascript
const inputRef = useRef(null);
useEffect(() => { inputRef.current.focus(); }, []);
return <input ref={inputRef} />;
```

**保存可变值**：存储一个在组件整个生命周期内持续存在的值，修改它不会触发重新渲染。适合保存定时器 ID、上一次的值、不需要触发渲染的中间状态等。

```javascript
const timerRef = useRef(null);
const prevCountRef = useRef(count);
```

**useRef 和 useState 的区别**：`useRef` 修改 `.current` 不会触发重新渲染，`useState` 会。

---

### 2.4 useMemo 和 useCallback 的区别？

`useMemo` 用于**缓存计算结果**，避免每次渲染都重新计算。

```javascript
const sortedList = useMemo(() => {
  return list.sort((a, b) => a.id - b.id);
}, [list]);
```

`useCallback` 用于**缓存函数引用**，避免每次渲染都创建新的函数实例。通常用于传递给子组件的回调函数，配合 `React.memo` 避免子组件不必要的重新渲染。

```javascript
const handleClick = useCallback(() => {
  setCount(prev => prev + 1);
}, []);
```

**区别**：`useMemo` 缓存的是值，`useCallback` 缓存的是函数。`useCallback(fn, deps)` 等价于 `useMemo(() => fn, deps)`。

**注意**：不要滥用，`useMemo` 和 `useCallback` 本身也有开销。只有计算量大或需要避免子组件重渲染时才使用。

---

### 2.5 自定义 Hook 怎么写？

自定义 Hook 就是一个以 `use` 开头的函数，内部可以使用其他 Hook，用于**复用有状态逻辑**。

```javascript
function useLocalStorage(key, initialValue) {
  const [value, setValue] = useState(() => {
    const stored = localStorage.getItem(key);
    return stored ? JSON.parse(stored) : initialValue;
  });

  useEffect(() => {
    localStorage.setItem(key, JSON.stringify(value));
  }, [key, value]);

  return [value, setValue];
}

// 使用
const [theme, setTheme] = useLocalStorage('theme', 'light');
```

**特点**：
- 每个使用自定义 Hook 的组件都有独立的状态
- 可以组合多个 Hook
- 可以接受参数，返回值
- 和普通函数的区别是内部可以调用其他 Hook

---

## 三、组件通信

### 3.1 React 组件通信有哪些方式？

**父子通信**：
- **Props**：父组件向子组件传递数据
- **回调函数**：子组件通过调用父组件传递的函数来通知父组件

**子父通信**：
- **回调函数**：父组件传一个函数给子组件，子组件调用时把数据传回去

**兄弟组件通信**：
- **状态提升**：把共享状态提升到共同的父组件中管理
- **Context**：跨层级传递数据，避免 props 逐层传递（prop drilling）

**跨层级通信**：
- **Context + useReducer**：创建全局状态
- **状态管理库**：Redux / Zustand / Jotai

**任意组件通信**：
- **Redux / Zustand**：全局状态管理
- **事件总线**：自定义事件系统（不推荐）

```javascript
// Context 示例
const ThemeContext = createContext('light');

// 祖先组件提供
<ThemeContext.Provider value={theme}>
  <App />
</ThemeContext.Provider>

// 后代组件消费
const theme = useContext(ThemeContext);
```

---

### 3.2 Redux 和 Zustand 有什么区别？

| 特性 | Redux | Zustand |
|------|-------|---------|
| 学习成本 | 较高（action/reducer/dispatch） | 很低，类似 useState |
| 样板代码 | 较多（action type、reducer、dispatch） | 极少 |
| 包体积 | 较大（~7KB） | 很小（~1KB） |
| 中间件 | Redux Thunk / Redux Saga | 内置，简单直接 |
| TypeScript | 需要额外配置 | 原生支持 |
| 趋势 | 仍是主流，但生态在简化 | 轻量级首选，越来越流行 |

**选择建议**：大型项目、团队有 Redux 经验的继续用 Redux（配合 Redux Toolkit 简化）。中小项目或新项目推荐 Zustand，简单高效。

```javascript
// Zustand 示例
const useStore = create((set) => ({
  count: 0,
  increment: () => set((state) => ({ count: state.count + 1 })),
}));

// 在组件中使用
const { count, increment } = useStore();
```

---

## 四、性能优化

### 4.1 React 性能优化有哪些手段？

**组件层面**：
- `React.memo`：浅比较 props，避免不必要的重渲染
- 合理拆分组件：让状态变化的影响范围最小化
- `useMemo` / `useCallback`：缓存计算结果和函数引用

**列表渲染**：
- 虚拟滚动（`react-window`、`react-virtualized`）处理大列表
- 列表 key 使用唯一标识，不用 index

**状态管理**：
- 避免不必要的状态提升
- 用 `useReducer` 替代多个 `useState`
- 状态拆分，避免一个状态变化导致大组件重渲染

**渲染优化**：
- 懒加载组件（`React.lazy` + `Suspense`）
- 避免在渲染中创建新对象/数组/函数

**构建优化**：
- 代码分割、路由懒加载
- Tree Shaking
- 生产模式构建

```javascript
// React.memo 配合 useCallback
const Child = React.memo(({ onClick }) => {
  return <button onClick={onClick}>Click</button>;
});

const handleClick = useCallback(() => { /* ... */ }, []);
<Child onClick={handleClick} />
```

---

### 4.2 什么时候该用 React.memo？

`React.memo` 是一个高阶组件，用于对 props 做浅比较，如果 props 没有变化就跳过重渲染。

**适合使用的场景**：
- 纯展示组件，渲染开销较大
- 被频繁重渲染的父组件包裹，但自身 props 很少变化
- 列表中的项组件

**不适合使用的场景**：
- props 经常变化的组件（加了 memo 也没用，反而增加浅比较开销）
- 渲染本身就很快的组件

**注意**：`React.memo` 只做浅比较。如果 props 中包含对象或函数，需要配合 `useMemo` / `useCallback` 使用，否则每次渲染都会生成新的引用，导致 memo 失效。

---

### 4.3 React.lazy 和 Suspense 是什么？

`React.lazy` 用于**动态导入组件**，实现组件级别的代码分割。`Suspense` 用于在组件加载过程中显示 fallback 内容。

```javascript
const HeavyComponent = React.lazy(() => import('./HeavyComponent'));

function App() {
  return (
    <Suspense fallback={<div>Loading...</div>}>
      <HeavyComponent />
    </Suspense>
  );
}
```

**适用场景**：
- 路由级别的懒加载
- 条件渲染的大组件（如弹窗、图表）
- 首屏不需要的非关键组件

---

## 五、进阶问题

### 5.1 React 的 Fiber 架构是什么？

Fiber 是 React 16 引入的新的协调引擎，它的核心目的是**让 React 的渲染过程可以中断和恢复**，从而实现优先级调度。

在 Fiber 之前，React 的更新是同步的、递归的，一旦开始就会一直执行到完。如果组件树很大，更新过程会长时间占用主线程，导致页面卡顿。

Fiber 把渲染工作拆分成很多小单元（fiber 节点），每个 fiber 节点对应一个组件或 DOM 元素。React 可以在执行完一个小单元后暂停，把控制权交还给浏览器处理高优先级任务（如用户输入、动画），然后再回来继续。

**Fiber 的两个阶段**：
- **Render 阶段**（可中断）：构建 Fiber 树，做 diff，标记需要更新的节点
- **Commit 阶段**（不可中断）：把变更提交到真实 DOM

**意义**：Fiber 架构是 Concurrent Mode（并发模式）的基础，让 React 能够实现 Suspense、流式 SSR 等高级特性。

---

### 5.2 什么是 React 的并发模式（Concurrent Mode）？

并发模式是 React 18 正式推出的特性，核心是让 React 的渲染过程可以**被打断、恢复、优先级排序**。

**主要特性**：

**自动批处理（Automatic Batching）**：React 18 之前只有 React 事件处理中的 setState 会批处理，setTimeout、Promise 中的不会。React 18 之后所有场景都会自动批处理，减少不必要的重渲染。

**Transitions**：用 `startTransition` 标记低优先级更新，让高优先级更新（如用户输入）优先处理。

```javascript
startTransition(() => {
  setSearchQuery(inputValue); // 低优先级：搜索结果可以稍后更新
});
setInputValue(inputValue);     // 高优先级：输入框立即更新
```

**Suspense for Data Fetching**：配合 Suspense 实现数据加载时的优雅降级。

**useId**：生成稳定的唯一 ID，用于 SSR 场景避免 hydration mismatch。

---

### 5.3 React 项目如何做权限控制？

**路由级权限**：在路由配置中添加权限字段，用路由守卫组件包裹需要权限的路由。

```javascript
const ProtectedRoute = ({ children, requiredPermission }) => {
  const user = useAuth();
  if (!user.hasPermission(requiredPermission)) {
    return <Navigate to="/403" />;
  }
  return children;
};

// 使用
<Route path="/admin" element={
  <ProtectedRoute requiredPermission="admin">
    <AdminPage />
  </ProtectedRoute>
} />
```

**按钮级权限**：封装权限组件或自定义 Hook。

```javascript
const WithPermission = ({ permission, children }) => {
  const user = useAuth();
  if (!user.hasPermission(permission)) return null;
  return children;
};

// 使用
<WithPermission permission="admin:delete">
  <Button onClick={handleDelete}>删除</Button>
</WithPermission>
```

**接口级权限**：在请求拦截器中统一添加 Token，在响应拦截器中处理 401/403。

---

### 5.4 React 的 SSR（服务端渲染）怎么做？

React SSR 的核心是在服务端把组件渲染成 HTML 字符串，发送给浏览器。

**实现方式**：

1. **Next.js**（推荐）：React 官方推荐的全栈框架，内置 SSR、SSG、ISR 等多种渲染模式，配置简单，生态丰富。

2. **手动搭建**：用 `react-dom/server` 的 `renderToString` 或 `renderToPipeableStream` 在 Node.js 中渲染组件。

**SSR 的优缺点**和 Vue SSR 类似：SEO 友好、首屏快，但开发复杂度高、服务器负载大。

**Next.js 的渲染模式**：
- **SSR**：每次请求都在服务端渲染
- **SSG（Static Site Generation）**：构建时生成静态 HTML，适合内容不变的场景
- **ISR（Incremental Static Regeneration）**：静态页面定期重新生成，兼顾性能和实时性

---

> **面试回答技巧**：React 面试重点在 Hooks 和性能优化。回答 Hooks 相关问题时，可以结合实际项目中的使用场景，比如"我在项目中用自定义 Hook 封装了请求逻辑，统一处理 loading 和 error 状态"。回答性能优化时，可以说"我在项目中用 React.memo 优化了列表项组件，配合虚拟滚动处理了上万条数据的渲染"。
