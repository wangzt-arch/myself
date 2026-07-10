import { useState, useRef, useEffect, useCallback } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import './index.scss';

const mockReplies = [
  `好的，我来帮你解答这个问题。

## 核心思路

这个问题可以从以下几个方面来分析：

1. **理解需求**：先明确你想要达成什么目标
2. **拆解问题**：把大问题拆成多个小步骤
3. **逐步实现**：从最简单的部分开始做起
4. **验证结果**：每一步都确认是否正确

\`\`\`javascript
// 示例代码
function greet(name) {
  return \`Hello, \${name}!\`;
}
\`\`\`

> 记住：复杂的问题都是由简单问题组合而成的。

希望这个回答对你有帮助！如果还有其他问题，随时问我。`,

  `这是一个很有意思的问题！让我详细解释一下。

### 什么是流式响应？

**流式响应（Streaming Response）** 是一种服务器端逐步推送数据的技术。相比传统的"一次性返回全部内容"，它有这些优势：

- **更快的首屏时间**：用户不需要等待全部内容生成
- **更好的体验**：像真人对话一样逐字输出
- **降低感知延迟**：即使总耗时相同，感觉上也更快

### 工作原理

1. 客户端发起请求
2. 服务器边生成边发送数据块
3. 客户端收到一块就渲染一块
4. 直到所有数据发送完毕

这种方式在 **AI 对话**、**实时翻译**、**大文件下载** 等场景中非常常用。

还有什么想了解的吗？`,

  `当然可以！我来给你一些建议。

## 提高代码质量的几个要点

### 1. 命名规范
- 变量名要有意义，避免 \`a\`、\`b\`、\`temp\` 这种
- 函数名用动词开头，如 \`getUserInfo\`、\`calculateTotal\`
- 常量全大写，如 \`MAX_COUNT\`

### 2. 代码结构
- 一个函数只做一件事（单一职责原则）
- 避免过深的嵌套（超过 3 层就要考虑重构）
- 合理使用注释，但不要写"废话注释"

### 3. 错误处理
- 不要忽略错误
- 使用 try-catch 捕获异常
- 给用户友好的错误提示

\`\`\`js
// ❌ 不好
if (data) {
  if (data.user) {
    if (data.user.name) {
      console.log(data.user.name);
    }
  }
}

// ✅ 好
const name = data?.user?.name;
if (name) console.log(name);
\`\`\`

坚持这些习惯，代码质量会稳步提升！`,

  `关于这个话题，我可以分享一些见解。

在软件开发中，**持续学习**是最重要的能力之一。技术日新月异，保持好奇心和学习热情非常关键。

### 推荐的学习路径

1. **打好基础** — 数据结构、算法、网络、操作系统
2. **精通一门语言** — 不要浅尝辄止
3. **了解设计模式** — 复用前人的智慧
4. **阅读优秀源码** — 学习高手怎么写代码
5. **动手实践** — 纸上得来终觉浅

> "Talk is cheap. Show me the code." — Linus Torvalds

你对哪个方向最感兴趣呢？我可以给你更具体的建议。`,

  `好问题！让我来详细解答。

首先，我们需要理解**问题的本质**。很多时候，答案就藏在问题本身里面。

### 分析步骤

- 第一步：明确问题是什么
- 第二步：收集相关信息
- 第三步：提出假设
- 第四步：验证假设
- 第五步：得出结论

这个过程看似简单，但实际应用中需要不断练习。

**记住**：没有愚蠢的问题，只有不去思考的头脑。

还有其他疑问吗？欢迎继续提问！`,
];

function getRandomReply(prompt) {
  const lower = prompt.toLowerCase();
  if (lower.includes('代码') || lower.includes('code') || lower.includes('编程')) {
    return mockReplies[2];
  }
  if (lower.includes('流') || lower.includes('stream') || lower.includes('实时')) {
    return mockReplies[1];
  }
  if (lower.includes('学习') || lower.includes('学习') || lower.includes('学')) {
    return mockReplies[3];
  }
  return mockReplies[Math.floor(Math.random() * mockReplies.length)];
}

async function simulateStreamResponse(prompt, onChunk, onComplete) {
  const reply = getRandomReply(prompt);
  const chars = reply.split('');
  let index = 0;

  const initialDelay = 400 + Math.random() * 400;
  await new Promise((resolve) => setTimeout(resolve, initialDelay));

  const timer = setInterval(() => {
    if (index >= chars.length) {
      clearInterval(timer);
      onComplete?.();
      return;
    }

    let chunk = chars[index];
    const remaining = chars.length - index;
    if (remaining > 3 && Math.random() < 0.3) {
      const extra = Math.min(Math.floor(Math.random() * 3), remaining - 1);
      chunk = chars.slice(index, index + 1 + extra).join('');
      index += extra;
    }

    onChunk?.(chunk);
    index++;
  }, 25 + Math.random() * 35);

  return () => clearInterval(timer);
}

const initialMessages = [
  {
    id: '1',
    role: 'assistant',
    content: '你好，我是你的 AI 助手。有什么可以帮到你吗？\n\n你可以问我任何问题，回答将实时流式输出。(只展示流式响应效果，实际并未调用 AI 接口)',
    isStreaming: false,
  },
];

export default function AIChat() {
  const [messages, setMessages] = useState(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isError, setIsError] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  const handleSend = async () => {
    const trimmed = inputValue.trim();
    if (!trimmed || isLoading) return;

    setInputValue('');
    setIsLoading(true);
    setIsError(false);

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: trimmed,
      isStreaming: false,
    };

    const assistantMessageId = (Date.now() + 1).toString();
    const assistantMessage = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      isStreaming: true,
    };

    setMessages((prev) => [...prev, userMessage, assistantMessage]);

    let completed = false;
    const finish = () => {
      if (completed) return;
      completed = true;
      setIsLoading(false);
    };

    try {
      simulateStreamResponse(
        trimmed,
        (chunk) => {
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === assistantMessageId
                ? { ...msg, content: msg.content + chunk }
                : msg
            )
          );
        },
        () => {
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === assistantMessageId ? { ...msg, isStreaming: false } : msg
            )
          );
          finish();
        }
      );
    } catch (error) {
      console.error('Stream error:', error);
      setIsError(true);
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? { ...msg, content: '抱歉，出现了问题，请稍后再试。', isStreaming: false }
            : msg
        )
      );
      finish();
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleRetry = () => {
    setIsError(false);
    const lastMsg = messages[messages.length - 1];
    if (lastMsg?.role === 'assistant' && !lastMsg.isStreaming) {
      setMessages((prev) => [
        ...prev.slice(0, -1),
        { ...lastMsg, content: '', isStreaming: true },
      ]);
      handleSend();
    }
  };

  const handleClear = () => {
    setMessages(initialMessages);
    setIsError(false);
  };

  return (
    <div className="ai-chat">
      <header className="ai-chat__header">
        <div className="ai-chat__title-wrap">
          <h1 className="ai-chat__title">AI 对话</h1>
          <p className="ai-chat__subtitle">实时流式响应</p>
        </div>
        <div className="ai-chat__actions">
          <div className={`ai-chat__status ${isLoading ? 'loading' : 'online'}`}>
            <span className="ai-chat__status-dot" />
            <span className="ai-chat__status-text">
              {isLoading ? '正在思考' : '在线'}
            </span>
          </div>
          <button className="ai-chat__clear-btn" onClick={handleClear} title="清空对话">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M3 6h18" />
              <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6" />
              <path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
            </svg>
          </button>
        </div>
      </header>

      <div className="ai-chat__messages">
        {messages.map((msg, index) => (
          <div
            key={msg.id}
            className={`ai-chat__msg ${msg.role} ${msg.isStreaming ? 'streaming' : ''}`}
            style={{ animationDelay: `${index * 0.04}s` }}
          >
            <div className="ai-chat__avatar">
              {msg.role === 'user' ? (
                <div className="ai-chat__avatar-icon user">
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
                    <circle cx="12" cy="7" r="4" />
                  </svg>
                </div>
              ) : (
                <div className="ai-chat__avatar-icon assistant">
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <path d="M12 8V4H8" />
                    <rect width="16" height="12" x="4" y="8" rx="2" />
                    <path d="M2 14h2" />
                    <path d="M20 14h2" />
                    <path d="M15 13v2" />
                    <path d="M9 13v2" />
                  </svg>
                </div>
              )}
            </div>
            <div className="ai-chat__bubble">
              <div className="ai-chat__content">
                {msg.role === 'assistant' ? (
                  <>
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>
                      {typeof msg.content === 'string' ? msg.content : ''}
                    </ReactMarkdown>
                    {msg.isStreaming && (
                      <span className="ai-chat__typing-dot" />
                    )}
                  </>
                ) : (
                  <p>{msg.content}</p>
                )}
              </div>
            </div>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      <div className="ai-chat__input-area">
        {isError && (
          <div className="ai-chat__error">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <circle cx="12" cy="12" r="10" />
              <line x1="12" y1="8" x2="12" y2="12" />
              <line x1="12" y1="16" x2="12.01" y2="16" />
            </svg>
            <span>连接失败</span>
            <button className="ai-chat__retry-btn" onClick={handleRetry}>
              重试
            </button>
          </div>
        )}
        <div className="ai-chat__input-wrap">
          <textarea
            className="ai-chat__input"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="输入消息..."
            disabled={isLoading}
            rows={1}
          />
          <button
            className="ai-chat__send-btn"
            onClick={handleSend}
            disabled={isLoading || !inputValue.trim()}
          >
            {isLoading ? (
              <span className="ai-chat__send-loading" />
            ) : (
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="22" y1="2" x2="11" y2="13" />
                <polygon points="22 2 15 22 11 13 2 9 22 2" />
              </svg>
            )}
          </button>
        </div>
        <p className="ai-chat__hint">Enter 发送 · Shift + Enter 换行</p>
      </div>
    </div>
  );
}
