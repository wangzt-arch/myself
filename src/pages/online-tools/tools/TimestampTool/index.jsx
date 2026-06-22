import { useState, useEffect } from 'react';
import './index.css';

/* 北京时间（UTC+8）格式化 */
const toBeijingString = (timestamp) => {
  if (!timestamp || isNaN(timestamp)) return '';
  const date = new Date(timestamp * 1000);
  const pad = (n) => String(n).padStart(2, '0');
  return `${date.getUTCFullYear()}-${pad(date.getUTCMonth() + 1)}-${pad(date.getUTCDate())} ${pad(date.getUTCHours() + 8 >= 24 ? date.getUTCHours() + 8 - 24 : date.getUTCHours() + 8)}:${pad(date.getUTCMinutes())}:${pad(date.getUTCSeconds())}`;
};

/* 另一种更简洁的实现：使用 Date + locale */
const formatBeijing = (timestamp) => {
  if (!timestamp || isNaN(timestamp)) return '';
  const ts = Number(timestamp);
  const date = ts > 1e12 ? new Date(ts) : new Date(ts * 1000);
  return date.toLocaleString('zh-CN', {
    timeZone: 'Asia/Shanghai',
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false,
  }).replace(/\//g, '-');
};

export default function TimestampTool() {
  const [now, setNow] = useState(Math.floor(Date.now() / 1000));

  /* 时间戳 → 北京时间 */
  const [tsInput, setTsInput] = useState('');
  const [tsResult, setTsResult] = useState('');
  const [tsError, setTsError] = useState('');

  /* 北京时间 → 时间戳 */
  const [dateInput, setDateInput] = useState('');
  const [timeInput, setTimeInput] = useState('');
  const [dtResult, setDtResult] = useState('');
  const [dtError, setDtError] = useState('');

  /* 当前时间 */
  useEffect(() => {
    const timer = setInterval(() => {
      setNow(Math.floor(Date.now() / 1000));
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  /* 初始化日期为当前北京时间 */
  useEffect(() => {
    const now = new Date();
    const beijing = new Date(now.toLocaleString('en-US', { timeZone: 'Asia/Shanghai' }));
    const y = beijing.getFullYear();
    const m = String(beijing.getMonth() + 1).padStart(2, '0');
    const d = String(beijing.getDate()).padStart(2, '0');
    const h = String(beijing.getHours()).padStart(2, '0');
    const min = String(beijing.getMinutes()).padStart(2, '0');
    setDateInput(`${y}-${m}-${d}`);
    setTimeInput(`${h}:${min}`);
  }, []);

  const handleTsConvert = () => {
    if (!tsInput.trim()) {
      setTsError('请输入时间戳');
      setTsResult('');
      return;
    }
    const ts = Number(tsInput.trim());
    if (!Number.isFinite(ts) || ts < 0) {
      setTsError('时间戳格式无效');
      setTsResult('');
      return;
    }
    const formatted = formatBeijing(ts);
    if (!formatted) {
      setTsError('转换失败');
      setTsResult('');
      return;
    }
    setTsError('');
    setTsResult(formatted);
  };

  const handleDtConvert = () => {
    if (!dateInput.trim()) {
      setDtError('请选择日期');
      setDtResult('');
      return;
    }
    const t = (timeInput || '00:00').padEnd(5, '0').padEnd(8, ':00');
    const iso = `${dateInput}T${t}`;
    const date = new Date(iso + '+08:00');
    if (isNaN(date.getTime())) {
      setDtError('日期时间格式无效');
      setDtResult('');
      return;
    }
    setDtError('');
    setDtResult(String(Math.floor(date.getTime() / 1000)));
  };

  const copy = (text, label) => {
    if (!text) return;
    navigator.clipboard.writeText(text);
  };

  return (
    <div className="ts-tool">
      {/* 当前时间面板 */}
      <div className="ts-now">
        <div className="ts-now__item">
          <div className="ts-now__label">当前时间戳</div>
          <div className="ts-now__value ts-now__value--mono">{now}</div>
          <button className="ts-now__copy" onClick={() => copy(String(now))}>复制</button>
        </div>
        <div className="ts-now__divider" />
        <div className="ts-now__item">
          <div className="ts-now__label">北京时间</div>
          <div className="ts-now__value ts-now__value--accent">{formatBeijing(now)}</div>
          <span className="ts-now__dot" />
        </div>
      </div>

      {/* 转换区域 */}
      <div className="ts-convert-area">
        {/* 时间戳 → 日期 */}
        <div className="ot-card">
          <div className="ot-card__header">
            <span className="ot-card__title">时间戳 → 北京时间</span>
          </div>
          <div className="ts-panel">
            <input
              type="text"
              className="ts-input"
              placeholder="输入 10 位秒级时间戳，例如 1761148800"
              value={tsInput}
              onChange={(e) => setTsInput(e.target.value)}
              onKeyDown={(e) => { if (e.key === 'Enter') handleTsConvert(); }}
              spellCheck={false}
            />
            <div className="ts-actions">
              <button className="ot-btn" onClick={() => { setTsInput(String(now)); setTsError(''); setTsResult(''); }}>
                使用当前
              </button>
              <button className="ot-btn ot-btn--primary" onClick={handleTsConvert}>
                转换
              </button>
            </div>
            <div className="ts-result">
              {tsError ? (
                <div className="ts-result__error">{tsError}</div>
              ) : tsResult ? (
                <>
                  <span className="ts-result__label">北京时间</span>
                  <span className="ts-result__value">{tsResult}</span>
                  <button className="ts-result__copy" onClick={() => copy(tsResult)}>复制</button>
                </>
              ) : null}
            </div>
          </div>
        </div>

        {/* 日期 → 时间戳 */}
        <div className="ot-card">
          <div className="ot-card__header">
            <span className="ot-card__title">北京时间 → 时间戳</span>
          </div>
          <div className="ts-panel">
            <div className="ts-datetime">
              <input
                type="date"
                className="ts-input ts-input--date"
                value={dateInput}
                onChange={(e) => setDateInput(e.target.value)}
              />
              <input
                type="time"
                className="ts-input ts-input--time"
                value={timeInput}
                onChange={(e) => setTimeInput(e.target.value)}
              />
            </div>
            <div className="ts-actions">
              <button className="ot-btn" onClick={() => {
                const now = new Date();
                const beijing = new Date(now.toLocaleString('en-US', { timeZone: 'Asia/Shanghai' }));
                const y = beijing.getFullYear();
                const m = String(beijing.getMonth() + 1).padStart(2, '0');
                const d = String(beijing.getDate()).padStart(2, '0');
                const h = String(beijing.getHours()).padStart(2, '0');
                const min = String(beijing.getMinutes()).padStart(2, '0');
                setDateInput(`${y}-${m}-${d}`);
                setTimeInput(`${h}:${min}`);
                setDtError('');
                setDtResult('');
              }}>
                使用当前
              </button>
              <button className="ot-btn ot-btn--primary" onClick={handleDtConvert}>
                转换
              </button>
            </div>
            <div className="ts-result">
              {dtError ? (
                <div className="ts-result__error">{dtError}</div>
              ) : dtResult ? (
                <>
                  <span className="ts-result__label">时间戳</span>
                  <span className="ts-result__value ts-result__value--mono">{dtResult}</span>
                  <button className="ts-result__copy" onClick={() => copy(dtResult)}>复制</button>
                </>
              ) : null}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
