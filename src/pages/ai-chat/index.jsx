import { useState, useRef, useEffect, useCallback } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import './index.scss';
import { streamChat, simulateLocalStream } from './api';
import { saveMessages, loadMessages, clearMessages } from './storage';

const initialMessages = (() => {
  // 优先从 localStorage 恢复
  const cached = loadMessages();
  if (cached && cached.length > 0) {
    return cached;
  }
  return [
    {
      id: '1',
      role: 'assistant',
      content: '你好，我是你的 AI 助手。有什么可以帮到你吗？\n\n基于 DeepSeek-R1 模型，支持深度推理和实时流式响应。',
      isStreaming: false,
    },
  ];
})();

export default function AIChat() {
  const [messages, setMessages] = useState(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isReasoning, setIsReasoning] = useState(false);
  const [isError, setIsError] = useState(false);
  const [useMock, setUseMock] = useState(false);
  const [stoppedMessageId, setStoppedMessageId] = useState(null);
  const messagesEndRef = useRef(null);
  const abortRef = useRef(null);
  const mockAbortRef = useRef(null);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  // 监听 messages 变化，自动持久化到 localStorage
  useEffect(() => {
    saveMessages(messages);
  }, [messages]);

  useEffect(() => {
    return () => {
      if (abortRef.current) {
        abortRef.current.abort();
      }
      if (mockAbortRef.current) {
        mockAbortRef.current();
      }
    };
  }, []);

  const startStream = (assistantMessageId, chatHistory, isContinue = false) => {
    const appendChunk = (chunk) => {
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId
            ? { ...msg, content: msg.content + chunk }
            : msg
        )
      );
    };

    const finishStream = () => {
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === assistantMessageId ? { ...msg, isStreaming: false } : msg
        )
      );
      setIsLoading(false);
      setIsReasoning(false);
      setStoppedMessageId(null);
    };

    if (useMock) {
      const lastUserMsg = isContinue
        ? '继续'
        : chatHistory[chatHistory.length - 1]?.content || '';
      mockAbortRef.current = simulateLocalStream(
        lastUserMsg,
        appendChunk,
        finishStream
      );
    } else {
      const controller = new AbortController();
      abortRef.current = controller;

      streamChat(
        chatHistory,
        controller.signal,
        appendChunk,
        () => setIsReasoning(true),
        () => {
          abortRef.current = null;
          finishStream();
        },
        (error) => {
          abortRef.current = null;
          console.warn('API 调用失败，降级到本地模拟:', error.message);
          setUseMock(true);
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === assistantMessageId ? { ...msg, content: '' } : msg
            )
          );
          const lastUserMsg = isContinue
            ? '继续'
            : chatHistory[chatHistory.length - 1]?.content || '';
          mockAbortRef.current = simulateLocalStream(
            lastUserMsg,
            appendChunk,
            finishStream
          );
        }
      );
    }
  };

  const handleSend = async () => {
    const trimmed = inputValue.trim();
    if (!trimmed || isLoading) return;

    setInputValue('');
    setIsLoading(true);
    setIsReasoning(false);
    setIsError(false);
    setStoppedMessageId(null);

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

    const chatHistory = [...messages, userMessage].map(({ role, content }) => ({
      role,
      content,
    }));

    startStream(assistantMessageId, chatHistory);
  };

  const handleStop = () => {
    // 中断当前请求
    if (abortRef.current) {
      abortRef.current.abort();
      abortRef.current = null;
    }
    if (mockAbortRef.current) {
      mockAbortRef.current();
      mockAbortRef.current = null;
    }

    // 找到当前正在流式输出的消息
    setMessages((prev) => {
      const streamingMsg = prev.find((msg) => msg.isStreaming && msg.role === 'assistant');
      if (streamingMsg) {
        setStoppedMessageId(streamingMsg.id);
      }
      return prev.map((msg) =>
        msg.isStreaming && msg.role === 'assistant'
          ? { ...msg, isStreaming: false }
          : msg
      );
    });

    setIsLoading(false);
    setIsReasoning(false);
  };

  const handleContinue = () => {
    if (!stoppedMessageId) return;

    setIsLoading(true);
    setIsReasoning(false);
    setStoppedMessageId(null);

    // 找到被中断的消息
    const stoppedMsg = messages.find((msg) => msg.id === stoppedMessageId);
    if (!stoppedMsg) return;

    // 更新消息为流式状态
    setMessages((prev) =>
      prev.map((msg) =>
        msg.id === stoppedMessageId ? { ...msg, isStreaming: true } : msg
      )
    );

    // 构建上下文：取到被中断消息为止的历史
    const historyIndex = messages.findIndex((msg) => msg.id === stoppedMessageId);
    const contextMessages = messages.slice(0, historyIndex).map(({ role, content }) => ({
      role,
      content,
    }));

    // 添加被中断的 assistant 消息（作为上下文的一部分）
    contextMessages.push({
      role: 'assistant',
      content: stoppedMsg.content,
    });

    // 添加 "继续" 用户消息
    contextMessages.push({
      role: 'user',
      content: '请继续',
    });

    startStream(stoppedMessageId, contextMessages, true);
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
    if (abortRef.current) {
      abortRef.current.abort();
      abortRef.current = null;
    }
    if (mockAbortRef.current) {
      mockAbortRef.current();
      mockAbortRef.current = null;
    }
    const welcome = [
      {
        id: '1',
        role: 'assistant',
        content: '你好，我是你的 AI 助手。有什么可以帮到你吗？\n\n基于 DeepSeek-R1 模型，支持深度推理和实时流式响应。',
        isStreaming: false,
      },
    ];
    setMessages(welcome);
    setIsError(false);
    setIsLoading(false);
    setIsReasoning(false);
    setStoppedMessageId(null);
    clearMessages();
  };

  return (
    <div className="ai-chat">
      <header className="ai-chat__header">
        <div className="ai-chat__title-wrap">
          <h1 className="ai-chat__title">AI 对话</h1>
          <p className="ai-chat__subtitle">
            {useMock ? '本地模拟模式' : 'DeepSeek-R1 · 深度推理'}
          </p>
        </div>
        <div className="ai-chat__actions">
          <div className={`ai-chat__status ${isLoading ? 'loading' : 'online'}`}>
            <span className="ai-chat__status-dot" />
            <span className="ai-chat__status-text">
              {isReasoning ? '深度思考中' : isLoading ? '正在思考' : useMock ? '模拟' : '在线'}
            </span>
          </div>
          <button
            className="ai-chat__mode-btn"
            onClick={() => setUseMock((v) => !v)}
            title={useMock ? '切换到 API 模式' : '切换到模拟模式'}
          >
            {useMock ? (
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2z" />
                <path d="M12 6v6l4 2" />
              </svg>
            ) : (
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <circle cx="12" cy="12" r="10" />
                <line x1="2" y1="12" x2="22" y2="12" />
                <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
              </svg>
            )}
          </button>
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
                    {msg.isStreaming && msg.content === '' && isReasoning ? (
                      <div className="ai-chat__reasoning">
                        <div className="ai-chat__reasoning-icon">
                          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                            <circle cx="12" cy="12" r="10" />
                            <path d="M12 16v-4" />
                            <path d="M12 8h.01" />
                          </svg>
                        </div>
                        <span>深度思考中...</span>
                      </div>
                    ) : (
                      <>
                        {msg.isStreaming ? (
                          <div className="ai-chat__stream-text">{msg.content || ''}</div>
                        ) : (
                          <ReactMarkdown remarkPlugins={[remarkGfm]}>
                            {typeof msg.content === 'string' ? msg.content : ''}
                          </ReactMarkdown>
                        )}
                        {msg.isStreaming && <span className="ai-chat__typing-dot" />}
                      </>
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

        {stoppedMessageId && !isLoading && (
          <div className="ai-chat__continue-hint">
            <span>回答已中断</span>
            <button className="ai-chat__continue-btn" onClick={handleContinue}>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <polygon points="5 3 19 12 5 21 5 3" />
              </svg>
              继续回答
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
          {isLoading ? (
            <button
              className="ai-chat__stop-btn"
              onClick={handleStop}
              title="停止生成"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <rect x="6" y="6" width="12" height="12" rx="2" />
              </svg>
              停止
            </button>
          ) : (
            <button
              className="ai-chat__send-btn"
              onClick={handleSend}
              disabled={!inputValue.trim()}
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="22" y1="2" x2="11" y2="13" />
                <polygon points="22 2 15 22 11 13 2 9 22 2" />
              </svg>
            </button>
          )}
        </div>
        <p className="ai-chat__hint">Enter 发送 · Shift + Enter 换行</p>
      </div>
    </div>
  );
}
