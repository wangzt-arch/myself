import React, { useEffect, useRef, useState } from "react";
import { dispatchCommand, QUICK_COMMANDS } from "./bridge";
import "./index.scss";

// AI 对话面板（右侧功能面板下方
function AiPanel({ isOpen, onToggle, engine }) {
  const [messages, setMessages] = useState([
    { role: "ai", text: "你好！我是地球智能助手 🌍\n告诉我你想做什么，例如：\n· 飞到北京\n· 放一个烟花\n· 观察卫星" },
  ]);
  const [input, setInput] = useState("");
  const scrollRef = useRef(null);

  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [messages]);

  const send = (text) => {
    const content = (text ?? input).trim();
    if (!content) return;
    const userMessage = { role: "user", text: content };
    setMessages((prev) => [...prev, userMessage]);
    setInput("");

    try {
      // 核心调度：本地 MCP 引擎执行工具
      const result = dispatchCommand(content, engine);
      const aiMessage = { role: "ai", text: result?.message || "执行完成" };
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      setMessages((prev) => [...prev, { role: "ai", text: `执行失败：${error.message || error}` }]);
    }
  };

  const onKeyDown = (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      send();
    }
  };

  return (
    <>
      {/* 收起时：只显示一个悬浮 AI 图标 */}
      {!isOpen && (
        <button className="ai-collapsed-btn" type="button" onClick={onToggle} title="AI 智能控制">
          🤖
        </button>
      )}

      <div className={isOpen ? "ai-panel" : "ai-panel ai-panel--collapsed"}>
        <div className="ai-panel__header">
          <div>
            <span>AI Assistant</span>
            <strong>智能控制</strong>
          </div>
          <button type="button" onClick={onToggle}>
            {isOpen ? "收起" : "展开"}
          </button>
        </div>

        {isOpen && (
          <div className="ai-panel__body">
            <div className="ai-chat" ref={scrollRef}>
              {messages.map((msg, idx) => (
                <div className={msg.role === "user" ? "ai-bubble ai-bubble--user" : "ai-bubble ai-bubble--ai"} key={idx}>
                  {msg.text.split("\n").map((line, i) => (
                    <div key={i}>{line}</div>
                  ))}
                </div>
              ))}
            </div>

            <div className="ai-quicks">
              {QUICK_COMMANDS.map((cmd) => (
                <button key={cmd.text} type="button" onClick={() => send(cmd.text)}>
                  {cmd.label}
                </button>
              ))}
            </div>

            <div className="ai-input">
              <input
                type="text"
                value={input}
                placeholder="例如：放一个烟花 / 飞到北京"
                onChange={(e) => setInput(e.target.value)}
                onKeyDown={onKeyDown}
              />
              <button type="button" onClick={() => send()}>发送</button>
            </div>
          </div>
        )}
      </div>
    </>
  );
}

export default AiPanel;
