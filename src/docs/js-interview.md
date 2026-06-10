# JS 面试题

> 面试口语化回答参考，以文字解释为主、代码为辅，由浅入深排列。每道题都按「一句话回答 → 展开说明 → 代码辅助」的结构组织，方便你在面试中自然地表达出来。

---

## 一、基础概念

### 1.1 JavaScript 有哪些数据类型？

JavaScript 的数据类型分为两大类：**原始类型**和**引用类型**。

原始类型一共有 7 种：`number`、`string`、`boolean`、`undefined`、`null`、`bigint`、`symbol`。它们的特点是值直接存储在栈上，比较的时候是按值比较。

引用类型主要包括对象、数组、函数等。它们存储的是内存地址的引用，比较的时候是按引用比较，所以两个内容相同的对象用 `===` 比较会返回 `false`。

有一个经典的坑就是 `typeof null` 返回的是 `"object"`，这不是设计意图，而是 JS 早期实现的一个 bug，一直保留到了现在。如果你想准确判断类型，可以用 `Object.prototype.toString.call()` 方法。

```javascript
typeof null;              // "object"（历史 bug）
Object.prototype.toString.call(null); // "[object Null]"（准确判断）
```

---

### 1.2 `==` 和 `===` 的区别？

简单来说，`===` 是严格相等，要求类型和值都相同才返回 `true`；`==` 是宽松相等，在类型不同的时候会自动做类型转换再比较。

比如 `'1' == 1` 会返回 `true`，因为字符串 `'1'` 被转成了数字 `1`。但 `'1' === 1` 返回 `false`，因为类型不同。

还有一个特殊情况是 `null == undefined` 返回 `true`，这是规范里规定的，但 `null === undefined` 返回 `false`。

实际开发中建议始终使用 `===`，避免隐式类型转换带来的意想不到的结果。

```javascript
'1' == 1;   // true（自动转换）
'1' === 1;  // false（严格比较）
null == undefined;  // true
null === undefined; // false
```

---

### 1.3 `let`、`const`、`var` 的区别？

这三个关键字都可以用来声明变量，但它们在作用域、变量提升和可变性上有明显区别。

`var` 是 ES5 时代的产物，它声明的是**函数作用域**的变量，而且存在**变量提升**，也就是说你可以在声明之前使用它，只不过值是 `undefined`。另外 `var` 允许重复声明同一个变量。

`let` 和 `const` 是 ES6 新增的，它们声明的是**块级作用域**的变量，只在所在的 `{}` 内有效。它们不存在变量提升，在声明之前访问会报错，这个区域叫做"暂时性死区"。它们也不允许重复声明。

`let` 和 `const` 的区别就是 `const` 声明之后不能重新赋值，但如果是对象或数组，内部属性还是可以修改的，因为 `const` 冻结的是引用地址而不是值本身。

日常开发中推荐默认用 `const`，需要重新赋值的时候用 `let`，不再使用 `var`。

---

### 1.4 什么是闭包？有什么应用场景？

闭包简单说就是：**一个函数能够记住并访问它定义时所在作用域的变量，即使这个函数在其他地方执行**。

举个例子，我在一个函数内部定义了另一个函数，内部函数引用了外部函数的变量，然后把内部函数返回出去。这时候外部函数虽然执行完了，但内部函数仍然持有那些变量的引用，这就是闭包。

闭包最常见的应用场景有：创建私有变量（模拟 encapsulation）、实现防抖和节流、柯里化、以及 React Hooks 中保持状态等。

不过闭包也有需要注意的地方，因为它会一直持有外部变量的引用，如果不及时释放，可能会导致内存泄漏。比如在组件卸载时记得清除定时器和事件监听。

```javascript
function createCounter() {
  let count = 0; // 这个变量被闭包"记住"了
  return {
    increment() { return ++count; },
    getCount() { return count; },
  };
}
const counter = createCounter();
counter.increment(); // 1
counter.increment(); // 2
counter.getCount();  // 2
```

---

### 1.5 `this` 的指向规则？

`this` 的指向取决于函数是怎么被调用的，主要有以下几种情况：

第一种是**默认绑定**，函数直接调用的时候，非严格模式下 `this` 指向 `window`，严格模式下指向 `undefined`。

第二种是**隐式绑定**，当函数作为对象的方法调用时，`this` 指向那个对象，比如 `obj.fn()` 中 `this` 就是 `obj`。

第三种是**显式绑定**，通过 `call`、`apply`、`bind` 手动指定 `this` 的指向。

第四种是 **`new` 绑定**，用 `new` 调用构造函数时，`this` 指向新创建的实例。

这四种的优先级从高到低是：`new` > 显式绑定 > 隐式绑定 > 默认绑定。

另外箭头函数比较特殊，它没有自己的 `this`，它的 `this` 是定义时外层作用域的 `this`，而且不能用 `call`、`apply` 去改变。

```javascript
const obj = {
  name: 'Tom',
  regular() { console.log(this.name); },  // this → obj
  arrow: () => console.log(this.name),    // this → 外层（不是 obj）
};
```
| 方法 | 执行时机 | 参数传递 | 返回值 | 适用场景 |
|------|---------|---------|--------|---------|
| **call** | 立即执行 | 逐个传递 | 函数返回值 | 参数较少，已知个数 |
| **apply** | 立即执行 | 数组传递 | 函数返回值 | 参数较多，动态个数 |
| **bind** | 返回新函数 | 逐个传递 | 新函数 | 延迟执行，事件绑定 |

---

## 二、原型与继承

### 2.1 说一下原型链？

JavaScript 是基于原型的语言，每个对象都有一个内部属性指向它的原型对象，原型对象也有自己的原型，这样一层一层往上找，直到最顶层的 `Object.prototype`，它的原型是 `null`，这条链路就叫原型链。

当我们访问一个对象的属性时，如果这个对象自身没有这个属性，引擎就会沿着原型链往上找，直到找到为止或者到 `null` 还没找到就返回 `undefined`。

举个例子，我用构造函数创建了一个实例，实例自身有 `name` 属性，但 `sayHi` 方法是定义在构造函数的 `prototype` 上的，实例通过原型链就能访问到这个方法。

```javascript
function Person(name) { this.name = name; }
Person.prototype.sayHi = function() { console.log('Hi, ' + this.name); };

const tom = new Person('Tom');
tom.sayHi(); // 'Hi, Tom'（通过原型链找到的）
tom.hasOwnProperty('sayHi'); // false（不是自身属性）
```

---

### 2.2 如何实现继承？

在 ES6 之前，继承通常通过原型链、构造函数借用、组合继承等方式实现，写起来比较繁琐。

ES6 引入了 `class` 关键字，让继承变得非常直观。本质上 `class` 是语法糖，底层还是基于原型链的，但写法上更清晰。

用 `extends` 关键字实现继承，子类的 `constructor` 里必须先调用 `super()` 来执行父类的构造函数。子类可以重写父类的方法，也可以通过 `super` 调用父类的方法。

```javascript
class Animal {
  constructor(name) { this.name = name; }
  speak() { return `${this.name} makes a sound.`; }
}

class Dog extends Animal {
  constructor(name) { super(name); }
  speak() { return `${this.name} barks.`; }
}
```

---

## 三、异步编程

### 3.1 事件循环（Event Loop）的执行顺序？

JavaScript 是单线程的，但它通过事件循环机制来处理异步操作。

执行顺序是这样的：首先执行所有的**同步代码**，同步代码执行完之后，会先检查**微任务队列**，把里面的任务全部执行完，然后再去执行**宏任务队列**里的一个任务，执行完再回来检查微任务，如此循环。

微任务包括 `Promise.then/catch/finally`、`MutationObserver`、`queueMicrotask` 等。

宏任务包括 `setTimeout`、`setInterval`、`I/O`、UI 渲染等。

所以即使 `setTimeout` 设置了 0 毫秒，它也会在所有同步代码和微任务之后才执行。

```javascript
console.log('1');                          // 同步
setTimeout(() => console.log('2'), 0);      // 宏任务
Promise.resolve().then(() => console.log('3')); // 微任务
console.log('4');                          // 同步
// 输出：1 → 4 → 3 → 2
```

---

### 3.2 Promise 的三种状态及特点？

Promise 有三种状态：**pending**（等待中）、**fulfilled**（已成功）、**rejected**（已失败）。

状态只能从 `pending` 变成 `fulfilled`，或者从 `pending` 变成 `rejected`，一旦改变就不可逆。

Promise 有几个常用的静态方法：`Promise.all` 等待所有成功才成功，有一个失败就失败；`Promise.allSettled` 等待所有完成，不管成功失败都会返回结果；`Promise.race` 返回最先完成的结果；`Promise.any` 返回最先成功的结果。

```javascript
// allSettled 可以拿到每个任务的成功/失败状态
Promise.allSettled([
  fetch('/api/user'),
  fetch('/api/posts'),
]).then((results) => {
  results.forEach((r) => console.log(r.status)); // "fulfilled" 或 "rejected"
});
```

---

### 3.3 async/await 的原理？

`async/await` 本质上是 Promise 的语法糖，让异步代码写起来像同步代码一样直观。

`async` 加在函数前面，表示这个函数返回一个 Promise。函数内部用 `await` 来等待一个 Promise 的结果，`await` 会暂停函数的执行，等 Promise 完成后才继续往下走。

错误处理用 `try/catch`，相当于 Promise 的 `.catch()`。

实际开发中最常见的模式是在 `async` 函数里用 `await Promise.all()` 来并行发多个请求，这样既清晰又高效。

```javascript
async function fetchData() {
  try {
    const res = await fetch('/api/data');
    return await res.json();
  } catch (err) {
    console.error('请求失败:', err);
  }
}
```

---

## 四、DOM 与事件

### 4.1 事件冒泡和事件捕获？

当一个事件发生时，它会在 DOM 树中传播，传播分为三个阶段：**捕获阶段**（从 `window` 向下到目标元素）→ **目标阶段**（到达目标元素）→ **冒泡阶段**（从目标元素向上回到 `window`）。

默认情况下，我们用 `addEventListener` 注册的事件是在冒泡阶段触发的。如果第三个参数传 `true`，就在捕获阶段触发。

实际开发中，如果我们想阻止事件继续传播，可以用 `e.stopPropagation()`；如果想阻止默认行为（比如阻止表单提交、阻止链接跳转），用 `e.preventDefault()`。

```javascript
// 第三个参数 true 表示在捕获阶段触发
parent.addEventListener('click', handler, true);
// 默认 false，在冒泡阶段触发
child.addEventListener('click', handler);
```

---

### 4.2 事件委托的原理？

事件委托就是利用事件冒泡机制，把子元素的事件监听器统一绑定到父元素上。当子元素触发事件时，事件会冒泡到父元素，我们在父元素的事件处理函数里通过 `e.target` 判断实际触发事件的元素，然后执行对应的逻辑。

这样做有两个好处：一是减少了事件监听器的数量，提升性能；二是动态添加的子元素不需要重新绑定事件，因为事件是委托在父元素上的。

```javascript
// 只在 ul 上绑定一个监听器，而不是每个 li 都绑定
document.querySelector('ul').addEventListener('click', (e) => {
  if (e.target.tagName === 'LI') {
    handleClick(e.target);
  }
});
```

---

## 五、ES6+ 特性

### 5.1 箭头函数和普通函数的区别？

箭头函数和普通函数主要有四个区别：

第一，`this` 的指向不同。普通函数的 `this` 取决于怎么调用它，而箭头函数没有自己的 `this`，它继承的是定义时外层作用域的 `this`，而且不能用 `call`、`apply` 去改变。

第二，箭头函数没有 `arguments` 对象，如果需要获取参数，用剩余参数 `...args` 代替。

第三，箭头函数不能用 `new` 调用，不能作为构造函数。

第四，箭头函数没有 `prototype` 属性。

所以箭头函数适合用在不需要 `this` 动态绑定的场景，比如数组回调、Promise 回调等。不适合用作对象的方法或者需要 `this` 指向调用者的场景。

```javascript
const obj = {
  name: 'Tom',
  regular() { console.log(this.name); },  // ✅ this → obj
  arrow: () => console.log(this.name),    // ❌ this 不是 obj
};
```

---

### 5.2 解构赋值有哪些用法？

解构赋值是 ES6 提供的一种从数组或对象中提取值的简洁语法。

数组解构可以按位置取值，支持跳过元素、设置默认值、用剩余参数收集剩下的元素。对象解构可以按属性名取值，支持重命名和设置默认值。函数参数也可以用解构，让参数更清晰。

实际开发中最常用的场景是：从 API 响应中提取需要的字段、函数参数解构、交换变量、从 Hook 中提取返回值等。

```javascript
// 对象解构 + 重命名 + 默认值
const { name: userName = 'Guest', age } = { age: 25 };
// 函数参数解构
function greet({ name, role = 'user' } = {}) { /* ... */ }
// 交换变量
let x = 1, y = 2;
[x, y] = [y, x];
```

---

### 5.3 展开运算符 `...` 的应用？

展开运算符 `...` 有两个方向的用法：**展开**和**收集**。

展开就是把数组或对象"拆开"，比如合并两个数组、合并两个对象、浅拷贝。收集就是和函数的剩余参数配合使用，把多余的参数收集成一个数组。

实际开发中非常常用，比如合并配置项、浅拷贝状态、把类数组转成真正的数组等。

```javascript
// 对象合并（后面的覆盖前面的）
const config = { ...defaults, theme: 'dark' };
// 数组合并
const merged = [...arr1, ...arr2];
// 剩余参数
function sum(...nums) { return nums.reduce((a, b) => a + b, 0); }
```

---

## 六、手写代码

### 6.1 手写防抖（debounce）

防抖的核心思想是：**事件触发后不立即执行，而是等一段时间，如果这段时间内又触发了，就重新计时**。

实现方式就是用一个变量保存定时器 ID，每次触发时先清除上一个定时器，再设置一个新的。只有最后一次触发后经过指定时间，函数才会真正执行。

典型场景是搜索框输入联想、窗口 resize 等。

```javascript
function debounce(fn, delay = 300) {
  let timer = null;
  return function (...args) {
    clearTimeout(timer);
    timer = setTimeout(() => fn.apply(this, args), delay);
  };
}
```

---

### 6.2 手写节流（throttle）

节流的核心思想是：**函数在一段时间内最多执行一次**，不管这段时间内触发了多少次。

实现方式是记录上一次执行的时间，每次触发时判断距离上次执行是否超过了指定间隔，超过了才执行。

典型场景是滚动事件监听、按钮防止重复提交等。

```javascript
function throttle(fn, interval = 300) {
  let lastTime = 0;
  return function (...args) {
    const now = Date.now();
    if (now - lastTime >= interval) {
      lastTime = now;
      fn.apply(this, args);
    }
  };
}
```

---

### 6.3 手写深拷贝

深拷贝就是创建一个和原对象完全独立的新对象，修改新对象不会影响原对象。

基本思路是递归遍历对象的每个属性，如果是原始类型就直接赋值，如果是对象或数组就递归拷贝。需要特别处理的有：`Date` 对象、`RegExp` 对象、循环引用（用 `WeakMap` 记录已拷贝的对象避免无限递归）。

```javascript
function deepClone(obj, cache = new WeakMap()) {
  if (obj === null || typeof obj !== 'object') return obj;
  if (obj instanceof Date) return new Date(obj);
  if (obj instanceof RegExp) return new RegExp(obj);
  if (cache.has(obj)) return cache.get(obj); // 处理循环引用

  const clone = Array.isArray(obj) ? [] : {};
  cache.set(obj, clone);
  for (const key of Object.keys(obj)) {
    clone[key] = deepClone(obj[key], cache);
  }
  return clone;
}
```

---

### 6.4 手写 Promise

手写 Promise 主要考察对 Promise 状态机制和 `then` 链式调用的理解。

核心要点：构造函数接收一个 `executor`，里面有两个参数 `resolve` 和 `reject`。初始状态是 `pending`，调用 `resolve` 变为 `fulfilled`，调用 `reject` 变为 `rejected`，状态一旦改变就不可逆。

`then` 方法需要返回一个新的 Promise 来实现链式调用。如果当前状态已经是 `fulfilled` 或 `rejected`，就直接执行回调；如果还是 `pending`，就把回调存起来，等状态改变时再执行。

```javascript
class MyPromise {
  constructor(executor) {
    this.status = 'pending';
    this.value = undefined;
    this.reason = undefined;
    this.onFulfilled = [];
    this.onRejected = [];

    const resolve = (value) => {
      if (this.status === 'pending') {
        this.status = 'fulfilled';
        this.value = value;
        this.onFulfilled.forEach((fn) => fn(value));
      }
    };
    const reject = (reason) => {
      if (this.status === 'pending') {
        this.status = 'rejected';
        this.reason = reason;
        this.onRejected.forEach((fn) => fn(reason));
      }
    };
    try { executor(resolve, reject); } catch (e) { reject(e); }
  }

  then(onFulfilled, onRejected) {
    onFulfilled = typeof onFulfilled === 'function' ? onFulfilled : (v) => v;
    onRejected = typeof onRejected === 'function' ? onRejected : (e) => { throw e; };

    if (this.status === 'fulfilled') {
      return new MyPromise((resolve) => resolve(onFulfilled(this.value)));
    }
    if (this.status === 'rejected') {
      return new MyPromise((_, reject) => reject(onRejected(this.reason)));
    }
    return new MyPromise((resolve, reject) => {
      this.onFulfilled.push((value) => resolve(onFulfilled(value)));
      this.onRejected.push((reason) => reject(onRejected(reason)));
    });
  }

  catch(onRejected) { return this.then(null, onRejected); }
}
```

---

## 七、进阶问题

### 7.1 如何理解 JS 的垃圾回收机制？

JavaScript 引擎会自动回收不再使用的内存，主要有两种策略：

**标记清除**是现代浏览器主流采用的方案。引擎从根对象（`window`、全局变量等）出发，标记所有可达的对象，没有被标记到的就是垃圾，会被回收。

**分代回收**是 V8 引擎的优化策略。把堆内存分为新生代和老生代，新创建的对象放在新生代，新生代空间小但 GC 频率高；存活时间长的对象会被移到老生代，老生代空间大但 GC 频率低。

还有一种叫**引用计数**的方案，通过记录被引用的次数来决定是否回收，但它无法处理循环引用的问题，所以现代浏览器已经不再使用。

---

### 7.2 什么是内存泄漏？如何避免？

内存泄漏就是那些**不再需要但仍然被引用着、无法被垃圾回收**的内存。

前端常见的内存泄漏场景有这几种：

第一种是**没有清除的定时器或事件监听器**。比如在组件里用 `setInterval` 但卸载时忘了 `clearInterval`，定时器回调里引用的变量就一直无法被回收。

第二种是**闭包持有大对象的引用**。如果闭包里引用了一个很大的数组或对象，而且这个闭包会长期存在，那个大对象也就一直占着内存。

第三种是**脱离 DOM 但仍被变量引用**。比如用 `removeChild` 删除了一个节点，但代码中还有变量指向它，这个 DOM 节点就无法被回收。

避免的方法就是：组件卸载时及时清除定时器和事件监听、不再需要的引用手动置为 `null`、对于临时性的键值对映射可以用 `WeakMap`，它的键是弱引用，不会阻止垃圾回收。

---

### 7.3 前端性能优化有哪些手段？

前端性能优化可以从几个维度来考虑：

**加载优化**方面：做代码分割和按需加载，减少首屏资源体积；用 Tree Shaking 去掉没用到的代码；静态资源上 CDN；开启 Gzip 或 Brotli 压缩。

**渲染优化**方面：长列表用虚拟滚动只渲染可视区域；频繁触发的事件用防抖节流；减少 DOM 操作避免重排重绘；可以用 CSS 的 `will-change` 提示浏览器提前优化。

**缓存策略**方面：利用 HTTP 强缓存和协商缓存减少重复请求；可以用 Service Worker 做离线缓存；不常变的数据存 localStorage。

**网络优化**方面：用 `preload` 预加载关键资源、用 `preconnect` 提前建立连接；图片用懒加载和 WebP 格式。

**代码层面**：避免不必要的组件重渲染，React 中可以用 `memo`、`useMemo`；计算密集的任务可以用 Web Worker 放到后台线程。

---

### 7.4 什么是 XSS 和 CSRF？如何防御？

**XSS** 全称是跨站脚本攻击，核心是**往页面里注入恶意 JavaScript 代码**，当其他用户浏览这个页面时，恶意代码就会在他们浏览器里执行。

比如攻击者在输入框里输入 `<script>document.cookie</script>`，如果后端没有做转义直接存入数据库，前端渲染时就会执行这段脚本，窃取用户的 Cookie。

防御手段主要有三种：一是对用户输入做 HTML 转义，把 `<`、`>` 等特殊字符转成实体；二是设置 CSP（Content Security Policy）头部，限制页面只能加载指定来源的脚本；三是给 Cookie 设置 `HttpOnly` 属性，这样 JS 就读不到了。

**CSRF** 全称是跨站请求伪造，核心是**利用用户已登录的身份，在用户不知情的情况下发送恶意请求**。

比如用户已经登录了银行网站，然后访问了攻击者的页面，攻击者的页面里有一个隐藏的表单自动提交到银行的转账接口，浏览器会自动带上用户的 Cookie，银行服务器就以为是用户本人的操作。

防御手段主要有：使用 CSRF Token，每次请求携带一个服务端下发的随机令牌，攻击者无法获取；设置 Cookie 的 `SameSite` 属性为 `Strict` 或 `Lax`，阻止跨站请求携带 Cookie；服务端验证请求的 `Origin` 或 `Referer` 头。

---

> **面试回答技巧**：先一句话给出结论，再展开细节说明，最后结合项目经验举一个实际例子。手写题先说思路再写代码，边写边解释。遇到不会的坦诚说明，然后给出你的思考方向，比硬编要好。
