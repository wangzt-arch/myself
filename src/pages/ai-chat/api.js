const API_URL = 'https://api.siliconflow.cn/v1/chat/completions';
const API_KEY = 'sk-xbjccnsbkazvwtzlthasaqtjivezcdtuptjjteuwltjfbgxl'
const MODEL = 'deepseek-ai/DeepSeek-R1-0528-Qwen3-8B';


const SYSTEM_PROMPT = `你是一个专业、友好的AI助手。请遵循以下原则回答问题：

简洁明了：避免冗余，直接回答核心问题,回答使用中文。`;

export async function streamChat(messages, signal, onChunk, onReasoning, onComplete, onError, onFinishReason) {
  try {
    const fullMessages = [
      { role: 'system', content: SYSTEM_PROMPT },
      ...messages.map(({ role, content }) => ({ role, content })),
    ];

    const response = await fetch(API_URL, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${API_KEY}`,
      },
      body: JSON.stringify({
        model: MODEL,
        messages: fullMessages,
        stream: true,
        max_tokens: 16384,
        temperature: 0.7,
        top_p: 0.9,
      }),
      signal,
    });

    if (!response.ok) {
      const errText = await response.text();
      throw new Error(`API 请求失败 (${response.status}): ${errText}`);
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    let lastFinishReason = null;

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed || !trimmed.startsWith('data: ')) continue;

        const data = trimmed.slice(6);
        if (data === '[DONE]') {
          // 如果是因为达到 token 限制而结束，通知调用方
          if (lastFinishReason === 'length' && onFinishReason) {
            onFinishReason('length');
          }
          onComplete?.();
          return;
        }

        try {
          const json = JSON.parse(data);
          const choice = json.choices?.[0];
          const delta = choice?.delta;

          // 记录 finish_reason
          if (choice?.finish_reason) {
            lastFinishReason = choice.finish_reason;
          }
          
          if (delta?.reasoning_content) {
            onReasoning?.(delta.reasoning_content);
          }
          
          if (delta?.content) {
            onChunk?.(delta.content);
          }
        } catch {
          // 忽略解析错误
        }
      }
    }

    if (buffer.trim().startsWith('data: ') && buffer.trim().slice(6) !== '[DONE]') {
      try {
        const json = JSON.parse(buffer.trim().slice(6));
        const choice = json.choices?.[0];
        const delta = choice?.delta;
        if (choice?.finish_reason) {
          lastFinishReason = choice.finish_reason;
        }
        if (delta?.reasoning_content) onReasoning?.(delta.reasoning_content);
        if (delta?.content) onChunk?.(delta.content);
      } catch {
        // 忽略
      }
    }

    // 如果是因为达到 token 限制而结束，通知调用方
    if (lastFinishReason === 'length' && onFinishReason) {
      onFinishReason('length');
    }
    onComplete?.();
  } catch (error) {
    if (error.name === 'AbortError') {
      // 用户主动中断，不算错误
      return;
    }
    onError?.(error);
  }
}

export const mockReplies = [
  `好的，我来帮你解答这个问题。

## 核心思路

这个问题可以从以下几个方面来分析：

1. **理解需求**：先明确你想要达成什么目标。
2. **拆解问题**：把大问题拆成多个小步骤。
3. **逐步实现**：从最简单的部分开始做起。
4. **验证结果**：每一步都确认是否正确。

希望这个回答对你有帮助！`,
];

export function getRandomReply(prompt) {
  return mockReplies[Math.floor(Math.random() * mockReplies.length)];
}

export function simulateLocalStream(prompt, onChunk, onComplete, onAbort) {
  const reply = getRandomReply(prompt);
  const chars = reply.split('');
  let index = 0;
  let aborted = false;

  const initialDelay = 400 + Math.random() * 400;

  const timer = setTimeout(() => {
    const interval = setInterval(() => {
      if (aborted) {
        clearInterval(interval);
        return;
      }

      if (index >= chars.length) {
        clearInterval(interval);
        onComplete?.();
        return;
      }

      let chunk = chars[index];
      const remaining = chars.length - index;
      if (remaining > 3 && Math.random() < 0.3) {
        const extra = Math.min(Math.floor(Math.random() * 4), remaining - 1);
        chunk = chars.slice(index, index + 1 + extra).join('');
        index += extra;
      }

      onChunk?.(chunk);
      index++;
    }, 20 + Math.random() * 30);
  }, initialDelay);

  return () => {
    aborted = true;
    clearTimeout(timer);
  };
}
