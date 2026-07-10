import { useState, useRef, useEffect, useCallback } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import './index.scss';
import { getRandomReply } from './mockData';

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
  const timerRef = useRef(null);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  useEffect(() => {
    return () => {
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
    };
  }, []);

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

    try {
      const reply = getRandomReply(trimmed);
      const chars = reply.split('');
      let index = 0;

      const initialDelay = 400 + Math.random() * 400;
      await new Promise((resolve) => setTimeout(resolve, initialDelay));

      timerRef.current = setInterval(() => {
        if (index >= chars.length) {
          clearInterval(timerRef.current);
          timerRef.current = null;
          setMessages((prev) =>
            prev.map((msg) =>
              msg.id === assistantMessageId ? { ...msg, isStreaming: false } : msg
            )
          );
          setIsLoading(false);
          return;
        }

        let chunk = chars[index];
        const remaining = chars.length - index;
        if (remaining > 3 && Math.random() < 0.3) {
          const extra = Math.min(Math.floor(Math.random() * 4), remaining - 1);
          chunk = chars.slice(index, index + 1 + extra).join('');
          index += extra;
        }

        setMessages((prev) =>
          prev.map((msg) =>
            msg.id === assistantMessageId
              ? { ...msg, content: msg.content + chunk }
              : msg
          )
        );
        index++;
      }, 20 + Math.random() * 30);
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
      setIsLoading(false);
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
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }
    setMessages(initialMessages);
    setIsError(false);
    setIsLoading(false);
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
                    {msg.isStreaming ? (
                      <div className="ai-chat__stream-text">{msg.content || ''}</div>
                    ) : (
                      <ReactMarkdown remarkPlugins={[remarkGfm]}>
                        {typeof msg.content === 'string' ? msg.content : ''}
                      </ReactMarkdown>
                    )}
                    {msg.isStreaming && <span className="ai-chat__typing-dot" />}
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
