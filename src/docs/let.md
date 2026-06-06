## 一、let 与 const

### 1.1 块级作用域

`let` 和 `const` 声明的变量具有块级作用域，只在所在的 `{}` 代码块内有效。

```javascript
{
  let a = 10;
  var b = 1;
}

a // ReferenceError: a is not defined
b // 1
```

`for` 循环的计数器非常适合使用 `let`：

```javascript
for (let i = 0; i < 10; i++) {
  // i 只在循环体内有效
}
console.log(i); // ReferenceError
```

经典闭包陷阱的解决：

```javascript
// var —— 所有函数共享同一个 i，最终都输出 10
var a = [];
for (var i = 0; i < 10; i++) {
  a[i] = function () { console.log(i); };
}
a[6](); // 10

// let —— 每次循环 i 都是新变量，输出对应的索引
var a = [];
for (let i = 0; i < 10; i++) {
  a[i] = function () { console.log(i); };
}
a[6](); // 6
```

### 1.2 不存在变量提升

`var` 会发生"变量提升"，而 `let`/`const` 不会：

```javascript
console.log(foo); // undefined（变量提升）
var foo = 2;

console.log(bar); // ReferenceError（不会提升）
let bar = 2;
```

### 1.3 暂时性死区（TDZ）

在变量声明之前访问，会进入"暂时性死区"：

```javascript
let tmp = 123;
if (true) {
  tmp = 'abc'; // ReferenceError
  let tmp;
}
```

`typeof` 也不再安全：

```javascript
typeof x; // ReferenceError
let x;
```

函数默认参数中的 TDZ：

```javascript
function bar(x = y, y = 2) {
  return [x, y];
}
bar(); // ReferenceError: y 在 x 的默认值中还未声明
```

### 1.4 不允许重复声明

```javascript
function func() {
  let a = 10;
  let a = 1; // SyntaxError
}

function func(arg) {
  let arg; // SyntaxError: 与参数重复
}
```

### 1.5 const 的本质

`const` 保证的是变量指向的内存地址不变，而非值本身不变：

```javascript
const foo = {};
foo.prop = 123;      // ✅ 可以修改对象属性
foo = {};            // ❌ TypeError: 不能重新赋值

const arr = [];
arr.push(1);         // ✅ 可以修改数组内容
arr = [2];           // ❌ TypeError
```

使用 `Object.freeze` 冻结对象（浅冻结）：

```javascript
const foo = Object.freeze({});
foo.prop = 123; // 严格模式下报错

// 深冻结递归实现
const deepFreeze = (obj) => {
  Object.freeze(obj);
  Object.keys(obj).forEach((key) => {
    if (typeof obj[key] === 'object' && obj[key] !== null) {
      deepFreeze(obj[key]);
    }
  });
  return obj;
};
```

---

## 二、解构赋值

### 2.1 数组解构

```javascript
const [a, b, c] = [1, 2, 3];

// 跳过元素
const [first, , third] = [1, 2, 3];

// 默认值
const [x = 1, y = 2] = [undefined, null]; // x=1, y=null（null 不触发默认值）

// 剩余元素
const [head, ...tail] = [1, 2, 3, 4]; // head=1, tail=[2,3,4]
```

### 2.2 对象解构

```javascript
const { name, age } = { name: 'Alice', age: 25 };

// 重命名
const { name: userName } = { name: 'Bob' }; // userName='Bob'

// 默认值
const { role = 'user' } = { name: 'Alice' }; // role='user'

// 嵌套解构
const { user: { email } } = { user: { email: 'a@b.com' } };

// 剩余属性
const { id, ...rest } = { id: 1, name: 'A', age: 20 }; // rest={name:'A', age:20}
```

### 2.3 函数参数解构

```javascript
// 对象参数解构 + 默认值
function createUser({ name, age = 18, role = 'user' } = {}) {
  return { name, age, role };
}
createUser({ name: 'Tom' }); // { name: 'Tom', age: 18, role: 'user' }
createUser();                // { name: undefined, age: 18, role: 'user' }

// 数组参数解构
function sum([a, b, c]) {
  return a + b + c;
}
sum([1, 2, 3]); // 6
```

### 2.4 字符串与数值解构

```javascript
const [a, b, c] = 'hello'; // a='h', b='e', c='l'

const { toString: s } = 123;
s.call(456); // "456"
```

---

## 三、展开运算符（Spread）与剩余参数（Rest）

### 3.1 数组展开

```javascript
const arr1 = [1, 2];
const arr2 = [...arr1, 3, 4]; // [1, 2, 3, 4]

// 数组拷贝（浅拷贝）
const copy = [...arr1];

// 数组合并
const merged = [...arr1, ...arr2];

// 与解构结合
const [first, ...rest] = [1, 2, 3, 4]; // first=1, rest=[2,3,4]
```

### 3.2 对象展开

```javascript
const obj1 = { a: 1, b: 2 };
const obj2 = { ...obj1, c: 3 }; // { a: 1, b: 2, c: 3 }

// 对象拷贝（浅拷贝）
const copy = { ...obj1 };

// 对象合并（后面的属性覆盖前面的）
const merged = { ...obj1, ...obj2, b: 99 };

// 与解构结合
const { x, ...remaining } = { x: 1, y: 2, z: 3 }; // remaining={y:2, z:3}
```

### 3.3 函数剩余参数

```javascript
function sum(...numbers) {
  return numbers.reduce((a, b) => a + b, 0);
}
sum(1, 2, 3, 4); // 10

// 剩余参数必须是最后一个
function log(prefix, ...messages) {
  console.log(`[${prefix}]`, ...messages);
}
```

> ⚠️ 剩余参数与 `arguments` 的区别：`arguments` 是类数组对象，剩余参数是真正的数组，可以使用数组方法。

---

## 四、模块化导入导出

### 4.1 命名导出与导入

```javascript
// math.js
export const PI = 3.14159;
export function add(a, b) {
  return a + b;
}
export class Calculator {
  // ...
}

// main.js
import { PI, add, Calculator } from './math.js';
import { PI as PI_VALUE, add as sum } from './math.js';
import * as math from './math.js'; // math.PI, math.add
```

### 4.2 默认导出与导入

```javascript
// utils.js
export default function greet(name) {
  return `Hello, ${name}!`;
}

// main.js
import greet from './utils.js';
import myGreet from './utils.js'; // 可以任意命名
```

### 4.3 混合导出

```javascript
// api.js
export const BASE_URL = 'https://api.example.com';
export default class ApiClient {
  // ...
}

// main.js
import ApiClient, { BASE_URL } from './api.js';
```

### 4.4 动态导入

```javascript
// 按需加载，返回 Promise
const module = await import('./heavy-module.js');
module.default();

// 条件导入
if (condition) {
  const { helper } = await import('./helpers.js');
  helper();
}
```

---

## 五、ES2020+ 新增声明特性

### 5.1 可选链操作符 `?.`

安全访问深层嵌套属性，避免 `Cannot read property of undefined`：

```javascript
const user = { profile: { email: 'a@b.com' } };

// 传统写法
const email = user && user.profile && user.profile.email;

// 可选链
const email = user?.profile?.email; // 'a@b.com'
const phone = user?.profile?.phone; // undefined（不报错）

// 函数调用
const result = obj?.method?.(); // 如果 method 不存在，返回 undefined

// 数组访问
const first = arr?.[0];
```

### 5.2 空值合并运算符 `??`

只在左侧为 `null` 或 `undefined` 时返回右侧值（与 `||` 不同，`0`、`''`、`false` 被视为有效值）：

```javascript
const value = 0;
const a = value || 100;    // 100（0 被视为 falsy）
const b = value ?? 100;    // 0（0 是有效值）

const name = null;
const c = name ?? 'Anonymous'; // 'Anonymous'

// 与可选链结合
const port = server?.config?.port ?? 3000;
```

### 5.3 逻辑赋值运算符

```javascript
// ||= 逻辑或赋值
let a = 0;
a ||= 10; // a = 10（0 是 falsy）

let b = 5;
b ||= 10; // b = 5（5 是 truthy，不赋值）

// &&= 逻辑与赋值
let c = 0;
c &&= 10; // c = 0（0 是 falsy，不赋值）

let d = 5;
d &&= 10; // d = 10（5 是 truthy）

// ??= 空值合并赋值
let e = null;
e ??= 'default'; // e = 'default'

let f = 0;
f ??= 'default'; // f = 0（0 不是 null/undefined）
```

### 5.4 BigInt

用于表示任意精度的整数：

```javascript
const big = 123456789012345678901234567890n;
const alsoBig = BigInt('123456789012345678901234567890');

big + 1n; // 可以运算
// big + 1; // ❌ TypeError: 不能混合 BigInt 和 Number
```

### 5.5 globalThis

统一获取全局对象，跨环境兼容：

```javascript
// 以前需要判断环境
const globalObj = (typeof window !== 'undefined')
  ? window
  : (typeof global !== 'undefined') ? global : self;

// ES2020 之后
const globalObj = globalThis; // 浏览器、Node、Web Worker 通用
```

---

## 六、Promise 与 async/await

### 6.1 Promise 基础

Promise 是异步编程的核心解决方案，表示一个异步操作的最终完成（或失败）及其结果值。

**三种状态：**

| 状态 | 含义 | 说明 |
|------|------|------|
| `pending` | 等待中 | 初始状态，既没有成功也没有失败 |
| `fulfilled` | 已成功 | 操作成功完成 |
| `rejected` | 已失败 | 操作失败 |

状态一旦改变就不可逆：`pending → fulfilled` 或 `pending → rejected`。

**基本用法：**

```javascript
const promise = new Promise((resolve, reject) => {
  // 异步操作
  setTimeout(() => {
    const success = true;
    if (success) {
      resolve('操作成功'); // 将状态变为 fulfilled
    } else {
      reject('操作失败');  // 将状态变为 rejected
    }
  }, 1000);
});

// 消费 Promise
promise
  .then((result) => console.log(result))   // '操作成功'
  .catch((error) => console.log(error));    // 捕获错误
```

**链式调用：**

```javascript
fetch('/api/user')
  .then((res) => res.json())
  .then((user) => fetch(`/api/posts?userId=${user.id}`))
  .then((res) => res.json())
  .then((posts) => console.log(posts))
  .catch((err) => console.error('请求链中出错:', err));
```

**静态方法：**

```javascript
// 等待所有完成（全部成功才成功，一个失败就失败）
Promise.all([p1, p2, p3])
  .then(([r1, r2, r3]) => console.log('全部完成'))
  .catch((err) => console.log('有一个失败了'));

// 等待所有完成（无论成功失败，全部结束后才返回）
Promise.allSettled([p1, p2, p3])
  .then((results) => {
    results.forEach((r) => {
      console.log(r.status); // 'fulfilled' 或 'rejected'
    });
  });

// 返回最先完成的结果（无论成功失败）
Promise.race([p1, p2])
  .then((result) => console.log('最先完成的:', result));

// 返回最先成功的结果（全部失败才失败）
Promise.any([p1, p2])
  .then((result) => console.log('最先成功的:', result));

// 快速创建一个已成功的 Promise
Promise.resolve(42).then((v) => console.log(v)); // 42

// 快速创建一个已失败的 Promise
Promise.reject(new Error('fail')).catch((e) => console.log(e.message)); // 'fail'
```

### 6.2 async/await 语法

`async/await` 是 Promise 的语法糖，让异步代码看起来像同步代码。

**async 函数：**

```javascript
// async 函数总是返回一个 Promise
async function getUser() {
  return { name: 'Alice' }; // 自动包装为 Promise.resolve({ name: 'Alice' })
}
getUser().then((user) => console.log(user.name)); // 'Alice'
```

**await 关键字：**

```javascript
async function fetchUser() {
  try {
    const response = await fetch('/api/user');   // 暂停执行，等待 Promise 完成
    const user = await response.json();          // 再次等待
    return user;
  } catch (error) {
    console.error('请求失败:', error);
    throw error;
  }
}
```

**并发请求：**

```javascript
// 串行（一个接一个，总耗时 = 各耗时之和）
async function serial() {
  const user = await fetchUser();
  const posts = await fetchPosts(user.id);
  const comments = await fetchComments(posts[0].id);
}

// 并行（同时发起，总耗时 = 最慢的那个）
async function parallel() {
  const [user, posts, comments] = await Promise.all([
    fetchUser(),
    fetchPosts(),
    fetchComments(),
  ]);
}
```

**循环中的 await：**

```javascript
// ❌ 串行执行（每次等上一个完成）
async function processItems(items) {
  for (const item of items) {
    await processItem(item); // 一个一个处理
  }
}

// ✅ 并行执行（同时处理所有）
async function processItems(items) {
  await Promise.all(items.map((item) => processItem(item)));
}

// ✅ 控制并发数量（例如最多同时 3 个）
async function processItems(items, limit = 3) {
  const results = [];
  for (let i = 0; i < items.length; i += limit) {
    const batch = items.slice(i, i + limit);
    const batchResults = await Promise.all(batch.map(processItem));
    results.push(...batchResults);
  }
  return results;
}
```

### 6.3 Promise vs async/await 对比

| 维度 | Promise | async/await |
|------|---------|-------------|
| **代码风格** | 链式调用 `.then().catch()` | 同步式写法，更直观 |
| **错误处理** | `.catch()` 统一捕获 | `try/catch` 块捕获 |
| **调试体验** | 难以在 `.then` 链中打断点 | 可以像同步代码一样打断点 |
| **中间值传递** | 链式传递，变量作用域受限 | 所有变量在同一作用域，随时可用 |
| **条件分支** | 嵌套 `.then`，可读性差 | `if/else` 直接写，逻辑清晰 |
| **底层关系** | 基础机制 | `async/await` 本质是 Promise 的语法糖 |
| **适用场景** | 简单的单步异步、`Promise.all` 等组合 | 多步异步流程、需要中间值的复杂逻辑 |

**同一逻辑的两种写法对比：**

```javascript
// Promise 写法 —— 链式调用，中间值需要层层传递
function getUserPosts(userId) {
  return fetch(`/api/users/${userId}`)
    .then((res) => res.json())
    .then((user) => {
      return fetch(`/api/posts?author=${user.name}`)
        .then((res) => res.json())
        .then((posts) => ({ user, posts }));
    })
    .catch((err) => console.error(err));
}

// async/await 写法 —— 平铺直叙，中间值随时可用
async function getUserPosts(userId) {
  try {
    const res = await fetch(`/api/users/${userId}`);
    const user = await res.json();

    // user 在这里可以直接使用，无需闭包传递
    const postRes = await fetch(`/api/posts?author=${user.name}`);
    const posts = await postRes.json();

    return { user, posts };
  } catch (err) {
    console.error(err);
  }
}
```

**错误处理对比：**

```javascript
// Promise —— 错误可能在链中丢失
fetch('/api/data')
  .then((res) => {
    // 如果这里抛出异常，后面的 .catch 能捕获
    return res.json();
  })
  .then((data) => {
    // 如果这里抛出异常，也能被捕获
    return data.map((item) => item.id);
  })
  .catch((err) => {
    // 统一捕获上面所有错误
    console.error('出错了:', err);
  });

// async/await —— try/catch 更符合直觉
async function getData() {
  try {
    const res = await fetch('/api/data');
    const data = await res.json();
    return data.map((item) => item.id);
  } catch (err) {
    // 所有错误都在这里捕获
    console.error('出错了:', err);
  }
}
```

> **选择建议**：简单的一两步异步用 `Promise.then` 即可；多步串联、需要中间值、有条件分支的复杂流程优先用 `async/await`；并行场景用 `Promise.all` / `Promise.allSettled`。两者可以混用——在 `async` 函数中用 `await Promise.all([...])` 是最常见的模式。

### 6.4 顶层 await（ES2022）

ES2022 允许在模块顶层直接使用 `await`，无需包裹在 `async` 函数中：

```javascript
// config.js —— 模块顶层直接 await
const response = await fetch('/api/config');
export const config = await response.json();

// main.js —— 导入时自动等待 config.js 加载完成
import { config } from './config.js';
console.log(config.apiUrl);
```

适用场景：模块初始化时需要异步获取配置、预加载数据等。注意顶层 await 只能在 ES Module 中使用，CommonJS 模块不支持。

---

## 七、最佳实践速查

| 场景 | 推荐写法 | 避免 |
|------|---------|------|
| 循环计数器 | `for (let i = 0; ...)` | `for (var i = 0; ...)` |
| 常量声明 | `const PI = 3.14` | `var PI = 3.14` |
| 对象浅拷贝 | `{ ...obj }` | `Object.assign({}, obj)` |
| 数组浅拷贝 | `[ ...arr ]` | `arr.slice()` |
| 函数参数默认值 | `function fn(x = 1)` | `x = x \|\| 1` |
| 安全访问属性 | `obj?.prop?.value` | `obj && obj.prop && obj.prop.value` |
| 默认值（含 0/''） | `value ?? defaultVal` | `value \|\| defaultVal` |
| 模块导入 | `import { fn } from './mod'` | `require('./mod')` |

---

> **总结**：从 ES6 开始，JavaScript 的变量声明体系变得更加严谨和表达力更强。优先使用 `const`，需要重新赋值时用 `let`，彻底告别 `var`。善用解构、展开和模块化导入，可以让代码更简洁、更易维护。
