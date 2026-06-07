# TypeScript 面试题

> TypeScript 相关的高频面试题，涵盖基础类型、泛型、高级特性、工程配置，以口语化回答为主，由浅入深排列。

---

## 一、基础概念

### 1.1 TypeScript 和 JavaScript 有什么区别？

TypeScript 是 JavaScript 的超集，在 JavaScript 的基础上增加了**静态类型系统**。简单说就是 TypeScript = JavaScript + 类型注解。

JavaScript 是动态类型语言，变量的类型在运行时才能确定，写代码时可以随意赋值，灵活性高但容易出错。TypeScript 在编译阶段就检查类型，类型不对直接报错，把很多运行时的 bug 提前到编译阶段发现。

**TypeScript 的优势**：

- **类型安全**：编译时发现类型错误，减少线上 bug
- **代码提示**：IDE 能根据类型信息提供精准的自动补全和提示
- **可读性更好**：类型注解本身就是一种文档，看代码就知道参数和返回值的类型
- **重构更安全**：修改接口或类型后，所有使用的地方都会报错，不会遗漏
- **大型项目必备**：团队协作时代码更规范，减少沟通成本

**TypeScript 不是银弹**：增加了编译步骤、学习成本、开发初期可能觉得类型注解繁琐。但对于中大型项目和团队协作，收益远大于成本。

---

### 1.2 TypeScript 的基本类型有哪些？

TypeScript 的类型分为**原始类型**和**引用类型**两大类。

**原始类型**：`number`、`string`、`boolean`、`null`、`undefined`、`bigint`、`symbol`。这些和 JavaScript 的原始类型一一对应。

```typescript
let age: number = 25;
let name: string = 'Tom';
let isStudent: boolean = true;
let empty: null = null;
let notDefined: undefined = undefined;
```

**引用类型**：`object`、`array`、`function`、`class`、`enum` 等。

```typescript
// 数组
let list: number[] = [1, 2, 3];
let list2: Array<number> = [1, 2, 3];

// 元组（固定长度和类型的数组）
let tuple: [string, number] = ['hello', 42];

// 对象
let user: { name: string; age: number } = { name: 'Tom', age: 25 };

// 函数
let greet: (name: string) => string = (name) => `Hello, ${name}`;
```

**特殊类型**：
- `any`：任意类型，跳过类型检查，尽量少用
- `unknown`：未知类型，比 `any` 安全，使用前必须进行类型检查
- `void`：没有返回值
- `never`：永远不会返回的类型（比如抛出异常的函数）
- `enum`：枚举类型

---

### 1.3 interface 和 type 有什么区别？

`interface` 和 `type` 都可以用来定义对象的形状，大部分场景下可以互换，但有一些区别。

**相同点**：都可以描述对象形状、都可以扩展、都支持泛型。

**不同点**：

| 特性 | `interface` | `type` |
|------|-----------|--------|
| 声明合并 | ✅ 同名 interface 自动合并 | ❌ 不允许重名 |
| 扩展方式 | `extends` 继承 | `&` 交叉类型 |
| 适用范围 | 只能描述对象形状 | 可以描述任何类型（联合、交叉、元组、函数等） |
| 计算属性 | ✅ 支持 | ❌ 不支持 |

```typescript
// interface 声明合并
interface User { name: string; }
interface User { age: number; }
// 合并后 User = { name: string; age: number; }

// interface 继承
interface Admin extends User { role: string; }

// type 交叉
type Admin = User & { role: string; };

// type 可以做 interface 做不到的事
type Status = 'active' | 'inactive'; // 联合类型
type Pair<T> = [T, T];               // 元组
type Fn = (x: number) => string;    // 函数类型
```

**选择建议**：定义对象形状用 `interface`（可扩展性好），定义联合类型、交叉类型、工具类型用 `type`。团队内统一规范即可。

---

### 1.4 什么是泛型？怎么用？

泛型就是**把类型参数化**，让函数、接口、类可以支持多种类型，而不是写死某一种。

打个比方，泛型就像是一个模板，你传入什么类型，它就变成什么类型。

**泛型函数**：

```typescript
// 不用泛型：每种类型都要写一个函数
function identityString(arg: string): string { return arg; }
function identityNumber(arg: number): number { return arg; }

// 用泛型：一个函数搞定所有类型
function identity<T>(arg: T): T { return arg; }

identity<string>('hello'); // 返回 string
identity<number>(42);       // 返回 number
// 也可以自动推断
identity('hello');          // 自动推断 T = string
```

**泛型接口**：

```typescript
interface ApiResponse<T> {
  code: number;
  message: string;
  data: T;
}

type UserResponse = ApiResponse<{ id: number; name: string }>;
type ListResponse = ApiResponse<number[]>;
```

**泛型约束**：用 `extends` 限制泛型的范围。

```typescript
function getLength<T extends { length: number }>(arg: T): number {
  return arg.length;
}

getLength('hello');    // ✅ string 有 length
getLength([1, 2, 3]); // ✅ 数组有 length
getLength(123);       // ❌ number 没有 length
```

**实际开发中泛型非常常用**：API 响应类型、工具函数、React 组件的 Props 类型、Hook 的返回值类型等。

---

### 1.5 什么是枚举？有几种？

枚举是用来定义一组命名常量的类型，让代码更可读。

**数字枚举**：

```typescript
enum Direction {
  Up = 0,    // 默认从 0 开始
  Down = 1,
  Left = 2,
  Right = 3,
}
```

**字符串枚举**：

```typescript
enum Status {
  Active = 'ACTIVE',
  Inactive = 'INACTIVE',
  Pending = 'PENDING',
}
```

**常量枚举**：用 `const enum` 定义，编译后会被内联替换，不会生成额外的 JavaScript 代码。

```typescript
const enum Colors {
  Red = '#ff0000',
  Blue = '#0000ff',
}
// 编译后：var color = '#ff0000'; （直接替换，没有 enum 对象）
```

**实际使用建议**：字符串枚举比数字枚举更可读，调试时能看到有意义的值。不过现在很多团队更倾向于用 `as const` + 联合类型来替代枚举，因为更简洁。

```typescript
// 替代枚举的写法
const Status = {
  Active: 'ACTIVE',
  Inactive: 'INACTIVE',
} as const;

type Status = typeof Status[keyof typeof Status];
```

---

## 二、进阶类型

### 2.1 联合类型和交叉类型有什么区别？

**联合类型（Union）**用 `|` 表示，表示值可以是多种类型中的某一种。

```typescript
let id: string | number;
id = 'abc';  // ✅
id = 123;    // ✅
id = true;   // ❌

// 联合类型的窄化
function printId(id: string | number) {
  if (typeof id === 'string') {
    console.log(id.toUpperCase()); // 这里 id 被窄化为 string
  } else {
    console.log(id.toFixed(2));   // 这里 id 被窄化为 number
  }
}
```

**交叉类型（Intersection）**用 `&` 表示，表示同时具有多种类型的特征。

```typescript
type Person = { name: string; age: number };
type Employee = { companyId: string; role: string };

type Worker = Person & Employee;
// Worker = { name: string; age: number; companyId: string; role: string; }
```

**区别**：联合类型是"或"的关系（A 或 B），交叉类型是"且"的关系（A 且 B）。

---

### 2.2 什么是类型守卫（Type Guard）？

类型守卫就是**在运行时检查类型，让 TypeScript 能够在特定代码块中窄化类型**。

常见的类型守卫方式：

**typeof**：

```typescript
function double(value: string | number) {
  if (typeof value === 'string') {
    return value.repeat(2); // value 被窄化为 string
  }
  return value * 2;          // value 被窄化为 number
}
```

**instanceof**：

```typescript
class Dog { bark() {} }
class Cat { meow() {} }

function sound(animal: Dog | Cat) {
  if (animal instanceof Dog) {
    animal.bark(); // Dog
  } else {
    animal.meow(); // Cat
  }
}
```

**in 操作符**：

```typescript
interface Fish { swim(): void; }
interface Bird { fly(): void; }

function move(pet: Fish | Bird) {
  if ('swim' in pet) {
    pet.swim(); // Fish
  } else {
    pet.fly();  // Bird
  }
}
```

**自定义类型守卫**：

```typescript
function isString(value: unknown): value is string {
  return typeof value === 'string';
}

if (isString(value)) {
  console.log(value.toUpperCase()); // value 被窄化为 string
}
```

---

### 2.3 什么是映射类型和条件类型？

**映射类型**：基于已有类型创建新类型，对每个属性做统一的转换。

```typescript
// 把所有属性变成可选
type Partial<T> = {
  [K in keyof T]?: T[K];
};

// 把所有属性变成只读
type Readonly<T> = {
  readonly [K in keyof T]: T[K];
};

// 实际使用
type User = { name: string; age: number };
type PartialUser = Partial<User>; // { name?: string; age?: number }
```

TypeScript 内置了很多映射类型工具：`Partial`、`Required`、`Readonly`、`Pick`、`Omit`、`Record` 等。

**条件类型**：根据条件选择不同的类型。

```typescript
type IsString<T> = T extends string ? 'yes' : 'no';

type A = IsString<string>;  // 'yes'
type B = IsString<number>;  // 'no'
```

**infer 关键字**：在条件类型中推断类型变量。

```typescript
// 提取函数的返回值类型
type ReturnType<T> = T extends (...args: any[]) => infer R ? R : never;

function getUser() { return { name: 'Tom', age: 25 }; }
type User = ReturnType<typeof getUser>; // { name: string; age: number }
```

---

### 2.4 常用的内置工具类型有哪些？

TypeScript 内置了很多实用的工具类型：

| 工具类型 | 作用 | 示例 |
|---------|------|------|
| `Partial<T>` | 所有属性变可选 | `Partial<User>` |
| `Required<T>` | 所有属性变必填 | `Required<User>` |
| `Readonly<T>` | 所有属性变只读 | `Readonly<User>` |
| `Pick<T, K>` | 选取部分属性 | `Pick<User, 'name'>` |
| `Omit<T, K>` | 排除部分属性 | `Omit<User, 'age'>` |
| `Record<K, V>` | 构造键值对类型 | `Record<string, number>` |
| `Exclude<T, U>` | 从联合类型中排除 | `Exclude<'a'\|'b', 'a'>` → `'b'` |
| `Extract<T, U>` | 从联合类型中提取 | `Extract<'a'\|'b', 'a'>` → `'a'` |
| `NonNullable<T>` | 排除 null 和 undefined | `NonNullable<string \| null>` |
| `ReturnType<T>` | 获取函数返回值类型 | `ReturnType<typeof fn>` |
| `Parameters<T>` | 获取函数参数类型 | `Parameters<typeof fn>` |

```typescript
interface User {
  id: number;
  name: string;
  age: number;
  email: string;
}

// 创建一个更新用的类型（排除 id，其他可选）
type UpdateUser = Partial<Omit<User, 'id'>>;
// { name?: string; age?: number; email?: string; }
```

---

## 三、配置与工程化

### 3.1 tsconfig.json 中有哪些重要配置？

`tsconfig.json` 是 TypeScript 项目的配置文件，常用的配置项有：

**编译选项**：
- `strict: true`：开启所有严格检查，推荐开启
- `target`：编译目标（如 `ES5`、`ES2015`、`ES2020`）
- `module`：模块系统（如 `ESNext`、`CommonJS`）
- `lib`：引入的类型库（如 `ES2020`、`DOM`）
- `jsx`：JSX 处理方式（React 项目用 `react-jsx`，Vue 项目用 `preserve`）

**路径相关**：
- `include`：包含的文件/目录
- `exclude`：排除的文件/目录
- `baseUrl`：模块解析的基础路径
- `paths`：路径别名映射

```json
{
  "compilerOptions": {
    "strict": true,
    "target": "ES2020",
    "module": "ESNext",
    "lib": ["ES2020", "DOM"],
    "jsx": "react-jsx",
    "baseUrl": ".",
    "paths": {
      "@/*": ["src/*"]
    }
  },
  "include": ["src"],
  "exclude": ["node_modules", "dist"]
}
```

---

### 3.2 TypeScript 和 JavaScript 混合开发怎么做？

TypeScript 支持渐进式迁移，不需要一次性把整个项目都改成 TypeScript。

**方案一：允许 JS 文件引入 TS 文件**。TypeScript 默认允许引入 JS 文件，JS 文件会被当作 `any` 类型处理。

**方案二：`allowJs` 配置**。在 `tsconfig.json` 中设置 `"allowJs": true`，让 TypeScript 编译器也能处理 JS 文件。

**方案三：文件后缀混用**。`.ts` 和 `.tsx` 是 TypeScript 文件，`.js` 和 `.jsx` 是 JavaScript 文件。可以逐步把 `.js` 文件重命名为 `.ts`。

**迁移建议**：
1. 先在 `tsconfig.json` 中开启 `allowJs` 和 `strict: false`
2. 新文件用 TypeScript 写
3. 逐步把旧文件改为 `.ts`，先加少量类型注解
4. 随着类型覆盖度提高，逐步开启 `strict` 模式的各个选项
5. 最终目标：`strict: true`

---

### 3.3 什么是声明文件（.d.ts）？有什么作用？

声明文件是 TypeScript 用来描述 JavaScript 库类型的文件，后缀是 `.d.ts`。

因为很多第三方库是用 JavaScript 写的，没有类型信息。声明文件告诉 TypeScript 这些库导出了什么、参数是什么类型、返回值是什么类型，这样 TypeScript 就能做类型检查和代码提示。

**常见场景**：

1. **第三方库没有类型**：安装 `@types/库名`，比如 `@types/lodash`、`@types/node`
2. **自己写的 JS 库**：手动编写 `.d.ts` 声明文件
3. **全局变量/模块**：用 `declare` 关键字声明

```typescript
// 声明一个全局变量
declare const GLOBAL_CONFIG: {
  apiUrl: string;
  version: string;
};

// 声明一个模块
declare module 'my-lib' {
  export function greet(name: string): string;
  export const version: string;
}
```

**DefinitelyTyped**：是 GitHub 上的一个仓库，存放了大量第三方库的声明文件，可以通过 `npm install @types/库名` 安装。

---

## 四、React + TypeScript

### 4.1 React 组件的 TypeScript 类型怎么写？

**函数组件**：

```typescript
// Props 类型用 interface 定义
interface ButtonProps {
  text: string;
  onClick: () => void;
  disabled?: boolean;         // 可选属性
  size?: 'small' | 'medium' | 'large';  // 联合类型
  children?: React.ReactNode;  // 子元素
}

const Button: React.FC<ButtonProps> = ({ text, onClick, disabled, size = 'medium', children }) => {
  return (
    <button onClick={onClick} disabled={disabled}>
      {text}
      {children}
    </button>
  );
};

// 更简洁的写法（推荐）
const Button = ({ text, onClick, disabled }: ButtonProps) => {
  return <button onClick={onClick} disabled={disabled}>{text}</button>;
};
```

**Hooks 类型**：

```typescript
// useState 自动推断类型
const [count, setCount] = useState(0);          // number
const [user, setUser] = useState<User | null>(null); // 需要显式指定

// useRef
const inputRef = useRef<HTMLInputElement>(null);  // DOM 引用
const timerRef = useRef<number | null>(null);    // 值引用

// useCallback
const handleClick = useCallback((id: number) => {
  // ...
}, []);

// 自定义 Hook
function useLocalStorage<T>(key: string, initialValue: T): [T, (value: T) => void] {
  // ...
}
```

---

### 4.2 如何给 API 请求定义类型？

**定义接口类型**：

```typescript
interface User {
  id: number;
  name: string;
  email: string;
  avatar?: string;  // 可选
}

// API 响应通用类型
interface ApiResponse<T> {
  code: number;
  message: string;
  data: T;
}

// 分页响应
interface PaginatedResponse<T> {
  list: T[];
  total: number;
  page: number;
  pageSize: number;
}
```

**在请求中使用**：

```typescript
async function fetchUsers(): Promise<ApiResponse<PaginatedResponse<User>>> {
  const res = await fetch('/api/users');
  return res.json();
}

// 使用
const result = await fetchUsers();
const users = result.data.list; // 类型自动推断为 User[]
```

---

## 五、Vue + TypeScript

### 5.1 Vue 3 组件的 TypeScript 类型怎么写？

**defineProps**：

```typescript
// 运行时声明
const props = defineProps({
  title: String,
  count: { type: Number, default: 0 },
});

// TypeScript 声明（推荐）
interface Props {
  title: string;
  count?: number;
}

const props = withDefaults(defineProps<Props>(), {
  count: 0,
});
```

**defineEmits**：

```typescript
const emit = defineEmits<{
  (e: 'change', value: string): void;
  (e: 'update', id: number, data: Partial<User>): void;
}>();
```

**ref 和 reactive**：

```typescript
const count = ref<number>(0);
const user = ref<User>({ name: 'Tom', age: 25 });

const state = reactive<{
  list: Item[];
  loading: boolean;
}>({
  list: [],
  loading: false,
});
```

---

### 5.2 Pinia 的 TypeScript 类型怎么写？

Pinia 天然支持 TypeScript，类型推导非常好。

```typescript
import { defineStore } from 'pinia';

interface UserState {
  name: string;
  token: string;
  role: string;
}

export const useUserStore = defineStore('user', {
  state: (): UserState => ({
    name: '',
    token: '',
    role: 'user',
  }),
  getters: {
    isAdmin: (state) => state.role === 'admin',
  },
  actions: {
    login(userData: UserState) {
      this.name = userData.name;
      this.token = userData.token;
      this.role = userData.role;
    },
  },
});
```

---

## 六、常见问题

### 6.1 any 和 unknown 有什么区别？

`any` 会跳过所有类型检查，可以赋值给任何类型，也可以从任何类型赋值。它基本上就是关闭了 TypeScript 的类型保护。

`unknown` 是类型安全的 `any`。你可以把任何值赋给 `unknown` 类型的变量，但在使用它之前，必须通过类型检查（类型守卫）来确定它的具体类型。

```typescript
let a: any = 'hello';
a = 123;
let b: number = a; // ✅ any 可以赋值给任何类型（不安全）

let u: unknown = 'hello';
u = 123;
let c: number = u; // ❌ unknown 不能直接赋值给其他类型

// 必须先检查类型
if (typeof u === 'number') {
  let c: number = u; // ✅
}
```

**选择建议**：尽量避免 `any`，如果确实不知道类型，用 `unknown` 更安全。

---

### 6.2 什么是类型断言？什么时候用？

类型断言就是**告诉 TypeScript"我比你更了解这个类型"**，让 TypeScript 按照你指定的类型来处理。

```typescript
// as 语法
const input = document.getElementById('myInput') as HTMLInputElement;
input.value = 'hello';

// 尖括号语法（JSX 中不能用）
const input = <HTMLInputElement>document.getElementById('myInput');
```

**使用场景**：

1. **DOM 操作**：`getElementById` 返回 `HTMLElement | null`，需要断言为具体的元素类型
2. **联合类型窄化**：当开发者确定类型但 TypeScript 无法自动推断时
3. **第三方库**：库的类型定义不完善时

**注意**：类型断言不进行运行时检查，如果断言错误，运行时仍然会报错。所以只在确定类型正确时才使用，不要用断言来"欺骗"编译器。

```typescript
// ❌ 错误的断言，运行时会报错
const num: number = 'hello' as any as number;

// ✅ 正确的用法
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
```

---

### 6.3 如何避免 TypeScript 常见的坑？

**1. 不要滥用 any**：用 `unknown` 替代，或者花时间定义正确的类型。

**2. 注意可选链和 null 检查**：TypeScript 的严格模式下，`null` 和 `undefined` 不能赋值给非空类型。

```typescript
// ✅ 正确处理 null
const user = data?.user ?? defaultUser;
if (user) {
  console.log(user.name);
}
```

**3. 避免类型断言绕过检查**：断言只是告诉编译器"相信我"，不改变运行时行为。

**4. 善用工具类型**：不要手动写冗长的类型，用 `Pick`、`Omit`、`Partial` 等工具类型组合。

**5. 泛型不要过度设计**：如果只有一个地方用到的类型，直接写死就行，不需要抽象成泛型。

**6. 开启 strict 模式**：`strict: true` 能帮你发现很多潜在问题，虽然初期会有一些报错需要处理，但长期来看收益很大。

---

> **面试回答技巧**：TypeScript 面试常问类型系统和实际应用。回答时先说概念，再举代码示例，最后结合项目经验。比如问到泛型，可以说"我在项目中用泛型封装了 API 请求函数，定义了 ApiResponse<T> 通用响应类型，这样每个接口只需要传入 data 的具体类型就行"。问到工程配置，可以讲你们项目的 tsconfig 配置策略和迁移经验。
