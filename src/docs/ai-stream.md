# 前端 AI 流式输出实现

本文详细介绍前端实现 AI 对话流式响应的完整技术方案，从发起请求到 UI 渲染的每一个技术环节。

## 技术选型

| 层级 | 技术 | 作用 |
|------|------|------|
| UI 框架 | React + JSX | 组件化界面，状态管理（useState/useEffect/useRef/useCallback） |
| 样式 | SCSS (BEM 命名) | 模块级样式隔离，不泄漏到其他页面 |
| Markdown 渲染 | react-markdown + remark-gfm | 支持代码块、表格、列表等富文本格式 |
| 流式解析 | ReadableStream + TextDecoder | 实时读取服务器推送的 SSE 数据流 |

---

## 核心流程拆解

### 1. 发起 SSE 请求

```javascript
const response = await fetch(API_URL, {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${API_KEY}`,
  },
  body: JSON.stringify({
    model: MODEL,
    messages: fullMessages,
    stream: true,           // 启用流式模式
    max_tokens: 2048,
    temperature: 0.7,
  }),
  signal,                   // AbortController 信号，用于中断
});
```

**关键参数说明：**

- **`stream: true`** — 告诉服务器不要等全部内容生成完毕再返回，而是边生成边推送。此时服务器返回的 `Content-Type` 为 `text/event-stream`，即 SSE 格式。
- **`signal`** — 传入 `AbortController.signal`，用户点击"停止"时调用 `controller.abort()`，浏览器立即终止 TCP 连接，服务器感知到连接断开也会停止生成。

---

### 2. 获取 ReadableStream

```javascript
const reader = response.body.getReader();
```

`response.body` 是一个 `ReadableStream` 对象。流式响应下，响应体不会一次性加载到内存，而是以**字节块（chunk）**的形式按需读取。

`getReader()` 创建一个读取器，独占该流。每次调用 `reader.read()` 返回一个 Promise：

```javascript
{ done: false, value: Uint8Array([...]) }   // 还有数据
{ done: true, value: undefined }             // 流结束
```

---

### 3. TextDecoder 流式解码

```javascript
const decoder = new TextDecoder();
let buffer = '';

while (true) {
  const { done, value } = await reader.read();
  if (done) break;
  
  buffer += decoder.decode(value, { stream: true });
}
```

**为什么不用 `response.text()`？**

`response.text()` 会等全部字节下载完毕再一次性转为字符串，失去了流式的意义。`TextDecoder` 可以**边收边解**。

**`{ stream: true }` 参数的关键作用：**

- 一个中文字符的 UTF-8 编码可能是 3 个字节
- 如果某个 chunk 的边界正好切在一个字符的中间（比如只收到前 2 个字节），`TextDecoder` 会把不完整的字节**暂存内部**，等下一次 `decode()` 时自动拼接
- 这样用户永远不会看到"乱码半个字"

---

### 4. 缓冲区处理（最易出 bug 的环节）

```javascript
const lines = buffer.split('\n');
buffer = lines.pop() || '';
```

**为什么需要 buffer？**

SSE 数据格式是**按行分隔**的：
```
data: {"choices":[{"delta":{"content":"你"}}]}
data: {"choices":[{"delta":{"content":"好"}}]}
data: [DONE]
```

但 `reader.read()` 返回的 chunk 是**任意字节边界**，不保证按行对齐。可能一次收到：
```
chunk 1: data: {"choices":[{"delta":{"content":"你"}}]}\ndata: {"choices":[{"delta":{"content":"好"}}]}\nda
chunk 2: ta: {"choices":[{"delta":{"content":"！"}}]}\ndata: [DONE]
```

**处理逻辑：**
1. `buffer.split('\n')` — 按换行分割
2. `lines.pop()` — 最后一行可能不完整（上面例子中的 `da`），把它留到 `buffer` 等下一个 chunk
3. 中间的完整行进入解析循环

---

### 5. 解析 SSE 数据行

```javascript
for (const line of lines) {
  const trimmed = line.trim();
  if (!trimmed || !trimmed.startsWith('data: ')) continue;

  const data = trimmed.slice(6);    // 去掉 "data: " 前缀
  if (data === '[DONE]') {
    onComplete?.();
    return;
  }
}
```

SSE 标准中，每行以 `data: ` 开头。过滤空行和非数据行后，提取 JSON 字符串。

**`[DONE]` 标记**：OpenAI 兼容流式协议约定，当所有内容发送完毕后，会发送一行 `data: [DONE]` 作为结束信号。

---

### 6. JSON 解析与内容提取

```javascript
try {
  const json = JSON.parse(data);
  const delta = json.choices?.[0]?.delta;
  
  if (delta?.reasoning_content) {
    onReasoning?.(delta.reasoning_content);   // DeepSeek-R1 的思考链
  }
  
  if (delta?.content) {
    onChunk?.(delta.content);                  // 正式回答内容
  }
} catch {
  // 忽略解析错误
}
```

**响应结构：**
```json
{
  "choices": [{
    "delta": {
      "content": "你",           // 普通模型只有这个
      "reasoning_content": "让我想想..."  // DeepSeek-R1 等推理模型特有
    }
  }]
}
```

**为什么用可选链 `?.`？**

流式过程中，某些 chunk 可能只包含 `role: "assistant"` 的空 delta，或只包含 `reasoning_content` 不含 `content`。用可选链防止 `undefined` 报错。

---

### 7. 处理尾部残留 buffer

```javascript
if (buffer.trim().startsWith('data: ') && buffer.trim().slice(6) !== '[DONE]') {
  try {
    const json = JSON.parse(buffer.trim().slice(6));
    const delta = json.choices?.[0]?.delta;
    if (delta?.reasoning_content) onReasoning?.(delta.reasoning_content);
    if (delta?.content) onChunk?.(delta.content);
  } catch {
    // 忽略
  }
}
```

循环结束后，`buffer` 里可能还残留最后一行数据（因为服务器发送的最后一个 chunk 通常不以 `\n` 结尾）。这段代码确保不遗漏最后一个 token。

---

### 8. 回调到 UI 的完整链路

```javascript
streamChat(
  chatHistory,
  controller.signal,
  (chunk) => {              // onChunk
    updateSession(sessionId, (s) => ({
      messages: s.messages.map((msg) =>
        msg.id === assistantMessageId
          ? { ...msg, content: msg.content + chunk }
          : msg
      ),
    }));
  },
  () => setIsReasoning(true),   // onReasoning
  () => finishStream(),         // onComplete
  (error) => { }                // onError
);
```

**数据流：**

```
硅基流动 API
    ↓ 发送 "你"
onChunk("你")
    ↓
updateSession() → setSessions()
    ↓
React 状态更新
    ↓
重新渲染当前会话的消息列表
    ↓
msg.content = "你" → <div>你<span class="cursor"/></div>
    ↓
DOM 更新，用户看到"你"字出现
    ↓ 重复此过程...
```

---

### 9. 中断机制

```javascript
const handleStop = () => {
  if (abortRef.current) {
    abortRef.current.abort();   // 触发 AbortError
    abortRef.current = null;
  }
}
```

**底层发生了什么：**

1. `controller.abort()` 发送中止信号
2. 浏览器关闭 HTTP 连接（TCP FIN）
3. `reader.read()` 抛出 `AbortError`
4. 捕获异常后静默返回（用户主动中止不算错误）
5. 服务器感知连接断开，停止 token 生成，节省算力

---

### 10. 降级方案（Mock 模式）

```javascript
export function simulateLocalStream(prompt, onChunk, onComplete, onAbort) {
  const reply = getRandomReply(prompt);
  const chars = reply.split('');
  let index = 0;
  let aborted = false;

  const timer = setTimeout(() => {
    const interval = setInterval(() => {
      if (aborted) { clearInterval(interval); return; }
      
      let chunk = chars[index];
      // 30% 概率一次输出 1~4 个字符，模拟真实 token 粒度
      if (remaining > 3 && Math.random() < 0.3) {
        const extra = Math.min(Math.floor(Math.random() * 4), remaining - 1);
        chunk = chars.slice(index, index + 1 + extra).join('');
        index += extra;
      }
      
      onChunk?.(chunk);
      index++;
    }, 20 + Math.random() * 30);
  }, 400 + Math.random() * 400);
}
```

**设计意图：**
- `onChunk` / `onComplete` 的回调签名与真实 API **完全一致**
- UI 层完全不用改，只需替换数据源
- 随机延迟 + 随机多字符输出，让 mock 效果更接近真实 LLM

---

## 完整时序图

```
时间轴 ───────────────────────────────────────────────►

用户                    前端                      硅基流动 API
 │                       │                             │
 │── 输入"你好" ────────►│                             │
 │                       │── fetch(stream:true) ──────►│
 │                       │                             │── 开始生成
 │                       │◄── data: {"delta":{"content":"你"}}
 │                       │── onChunk("你")             │
 │                       │── setSessions(content="你") │
 │                       │── 渲染 DOM                  │
 │                       │◄── data: {"delta":{"content":"好"}}
 │                       │── onChunk("好")             │
 │                       │── setSessions(content="你好")│
 │                       │── 渲染 DOM                  │
 │                       │          ...                │
 │                       │◄── data: [DONE]             │
 │                       │── onComplete()              │
 │                       │                             │── 停止生成
 │                       │                             │
 │◄── 看到完整回答 ──────│                             │
```

---

## 性能优化

| 优化点 | 实现 |
|--------|------|
| 批量更新 | `requestAnimationFrame` 缓冲高频 setState |
| 流式期间跳过 Markdown 解析 | 用纯文本显示，结束后再切 `ReactMarkdown` |
| 历史消息防重渲染 | `React.memo` + 自定义比较函数 |
| 中断控制 | `AbortController` 支持用户主动停止生成 |

**为什么流式期间跳过 Markdown 解析？**

`ReactMarkdown` 每收到一个字符就重新解析整个字符串，是最大性能瓶颈。流式 300 字会触发 300 次解析。改为流式期间用纯文本 + 结束后一次性渲染，解析次数从 300 次降到 1 次。

---

## 关键技术选型理由

| 技术 | 为什么选它 |
|------|-----------|
| `fetch` + `ReadableStream` | 浏览器原生，零依赖，比 `XMLHttpRequest` 更现代的流式 API |
| `TextDecoder` | 专门处理字节到字符串的解码，支持流式模式防乱码 |
| SSE 而不是 WebSocket | LLM 推理是单向推送（服务器→客户端），SSE 更轻量，自动基于 HTTP |
| `AbortController` | 浏览器标准中断机制，能真正关闭 TCP 连接，不只是前端忽略数据 |
| `setInterval` (mock) | 简单可控，不需要引入复杂的事件源模拟库 |

---

## 模型与平台

| 项目 | 配置 |
|------|------|
| 模型 | `deepseek-ai/DeepSeek-R1-0528-Qwen3-8B` |
| 平台 | 硅基流动（SiliconFlow） |
| 接口格式 | OpenAI 兼容 `/v1/chat/completions` |
| 传输协议 | SSE（Server-Sent Events） |
| 特殊参数 | `stream: true`, `temperature: 0.7` |

**DeepSeek-R1 特性：**
- 支持 `reasoning_content` 字段（思考链输出）
- 流式期间先返回推理过程，再返回正式回答

---

## 多会话架构

| 功能 | 实现 |
|------|------|
| 会话存储 | `sessions[]` 数组 + `currentSessionId` |
| 持久化 | localStorage（最多 50 个会话，30 天有效期） |
| 自动命名 | 取第一条用户消息前 20 字作为标题 |
| 日期分组 | 按 `今天/昨天/过去7天/过去30天/更早` 自动归类 |

---

## 交互功能清单

- **停止生成** — `AbortController.abort()` 中断流式请求
- **继续回答** — 把截断内容作为上下文，发送"请继续"续接
- **新建/切换/删除对话** — 左侧边栏管理多会话
- **模式切换** — API 模式 ↔ 本地模拟模式一键切换
- **本地缓存** — localStorage 持久化，刷新页面不丢失
- **深度思考状态** — DeepSeek-R1 推理过程单独显示
