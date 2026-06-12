import React, { useState, useRef, useEffect, useCallback } from "react";
import { getTranslate } from "../../api";
import "./index.scss";

const STORAGE_KEY = "translate_history";
const MAX_HISTORY = 10;

function loadHistory() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    return raw ? JSON.parse(raw) : [];
  } catch {
    return [];
  }
}

function saveHistory(list) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(list.slice(0, MAX_HISTORY)));
  } catch { /* ignore */ }
}

function Translate() {
  const [transRes, setTransRes] = useState("");
  const [language, setLanguage] = useState("en");
  const [transInp, setTransInp] = useState("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const [copyTip, setCopyTip] = useState("复制");
  const [history, setHistory] = useState(loadHistory);
  const inputRef = useRef(null);
  const timerRef = useRef(null);

  const doTranslate = useCallback(async (text, lang) => {
    if (!text.trim()) {
      setTransRes("");
      setError("");
      return;
    }
    setLoading(true);
    setError("");
    try {
      const res = await getTranslate(text, lang);
      setTransRes(res.data.target);
      setHistory((prev) => {
        const item = { source: text, target: res.data.target, lang, time: Date.now() };
        const next = [item, ...prev.filter((h) => h.source !== text)];
        saveHistory(next);
        return next.slice(0, MAX_HISTORY);
      });
    } catch {
      setError("翻译请求失败，请检查网络后重试");
    } finally {
      setLoading(false);
    }
  }, []);

  const inputWords = (e) => {
    const val = e.target.value;
    setTransInp(val);
    setError("");
    if (timerRef.current) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => doTranslate(val, language), 600);
  };

  const languageChange = (e) => {
    setLanguage(e.target.value);
  };

  const swapLanguage = () => {
    const next = language === "en" ? "zh" : "en";
    setLanguage(next);
    if (transRes) {
      setTransInp(transRes);
      setTransRes("");
      setTimeout(() => doTranslate(transRes, next), 100);
    }
  };

  const clear = () => {
    if (inputRef.current) inputRef.current.value = "";
    setTransInp("");
    setTransRes("");
    setError("");
    if (timerRef.current) clearTimeout(timerRef.current);
    inputRef.current?.focus();
  };

  const copy = async () => {
    if (!transRes) return;
    try {
      await navigator.clipboard.writeText(transRes);
      setCopyTip("已复制");
      setTimeout(() => setCopyTip("复制"), 1500);
    } catch {
      const el = document.createElement("textarea");
      el.value = transRes;
      document.body.appendChild(el);
      el.select();
      document.execCommand("copy");
      document.body.removeChild(el);
      setCopyTip("已复制");
      setTimeout(() => setCopyTip("复制"), 1500);
    }
  };

  const onKeyDownEnter = (e) => {
    if (e.code === "Enter" && !e.shiftKey) {
      e.preventDefault();
      if (timerRef.current) clearTimeout(timerRef.current);
      doTranslate(transInp, language);
    }
  };

  const applyHistory = (item) => {
    setTransInp(item.source);
    setLanguage(item.lang);
    setTransRes(item.target);
    setError("");
  };

  useEffect(() => {
    return () => {
      if (timerRef.current) clearTimeout(timerRef.current);
    };
  }, []);

  return (
    <div className="translate-page">
      <main className="translate-main">
        {/* 语言工具栏 */}
        <div className="translate-toolbar">
          <span className="pill pill--primary">
            自动检测
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
              <path d="M5 12h14M12 5l7 7-7 7" />
            </svg>
          </span>
          <select className="toolbar-select" value={language} onChange={languageChange}>
            <option value="en">英文</option>
            <option value="zh">中文</option>
          </select>
          <button className="toolbar-swap" onClick={swapLanguage} title="切换语言">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M7 16V4m0 0L3 8m4-4l4 4" />
              <path d="M17 8v12m0 0l4-4m-4 4l-4-4" />
            </svg>
            <span>切换</span>
          </button>
        </div>

        {/* 翻译区域 */}
        <div className="translate-box">
          {/* 输入区 */}
          <div className="translate-pane translate-pane--input">
            <textarea
              ref={inputRef}
              className="translate-textarea"
              placeholder="请输入要翻译的文本…"
              rows="6"
              onChange={inputWords}
              onKeyDown={onKeyDownEnter}
              autoFocus
            />
            <div className="pane-footer">
              <span className="char-count">{transInp.length} 字</span>
              {transInp && (
                <button className="pane-action pane-action--clear" onClick={clear} title="清除">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <line x1="18" y1="6" x2="6" y2="18" />
                    <line x1="6" y1="6" x2="18" y2="18" />
                  </svg>
                </button>
              )}
            </div>
          </div>

          {/* 输出区 */}
          <div className="translate-pane translate-pane--output">
            {loading ? (
              <div className="translate-loading">
                <span className="loading-spinner" />
                翻译中…
              </div>
            ) : error ? (
              <div className="translate-error">{error}</div>
            ) : transRes ? (
              <div className="result-card">
                <p className="result-text">{transRes}</p>
                <button className="result-copy" onClick={copy}>
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                    <rect x="9" y="9" width="13" height="13" rx="2" ry="2" />
                    <path d="M5 15H4a2 2 0 01-2-2V4a2 2 0 012-2h9a2 2 0 012 2v1" />
                  </svg>
                  {copyTip}
                </button>
              </div>
            ) : (
              <div className="translate-placeholder">翻译结果将在这里显示</div>
            )}
          </div>
        </div>

        {/* 翻译历史 */}
        {history.length > 0 && (
          <section className="translate-history">
            <div className="history-heading">
              <span className="pill pill--primary pill--sm">翻译历史</span>
              <button
                className="pill pill--primary pill--sm"
                onClick={() => { setHistory([]); localStorage.removeItem(STORAGE_KEY); }}
              >
                清空
              </button>
            </div>
            <ul className="history-list">
              {history.map((item) => (
                <li key={item.time} className="history-item" onClick={() => applyHistory(item)}>
                  <span className="history-source">{item.source}</span>
                  <span className="history-eq">=</span>
                  <span className="history-target">{item.target}</span>
                </li>
              ))}
            </ul>
          </section>
        )}

        {/* 快捷键提示 */}
        <p className="translate-hint">按 <kbd>Enter</kbd> 立即翻译 · 输入后自动翻译</p>
      </main>
    </div>
  );
}

export default Translate;
