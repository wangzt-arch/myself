import { useState, useEffect } from 'react';
import './index.css';

export default function RegexTool() {
  const [pattern, setPattern] = useState('');
  const [flags, setFlags] = useState({ g: true, i: false, m: false });
  const [testText, setTestText] = useState('');
  const [matches, setMatches] = useState([]);

  const flagList = [
    { key: 'g', label: 'g', desc: '全局匹配' },
    { key: 'i', label: 'i', desc: '忽略大小写' },
    { key: 'm', label: 'm', desc: '多行模式' },
  ];

  const quickPatterns = [
    { label: '邮箱', pattern: '[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}' },
    { label: '手机号', pattern: '1[3-9]\\d{9}' },
    { label: 'IP地址', pattern: '\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}' },
    { label: 'URL', pattern: 'https?://[\\w\\-.]+(:\\d+)?(/[\\w\\-./?%&=]*)?' },
    { label: '身份证', pattern: '\\d{17}[\\dXx]' },
    { label: '日期', pattern: '\\d{4}-\\d{2}-\\d{2}' },
    { label: '时间', pattern: '\\d{2}:\\d{2}:\\d{2}' },
    { label: '中文', pattern: '[\\u4e00-\\u9fa5]+' },
    { label: '数字', pattern: '-?\\d+(\\.\\d+)?' },
    { label: '用户名', pattern: '[a-zA-Z][a-zA-Z0-9_]{3,15}' },
  ];

  const toggleFlag = (flag) => setFlags((prev) => ({ ...prev, [flag]: !prev[flag] }));
  const applyPattern = (p) => setPattern(p);

  useEffect(() => {
    if (!pattern || !testText) { setMatches([]); return; }
    try {
      const flagStr = Object.entries(flags).filter(([, v]) => v).map(([k]) => k).join('');
      const regex = new RegExp(pattern, flagStr);
      const list = [];
      if (flags.g) {
        let m;
        while ((m = regex.exec(testText)) !== null) {
          list.push({ index: list.length + 1, match: m[0], groups: m.slice(1) });
          if (m[0].length === 0) regex.lastIndex++;
        }
      } else {
        const m = regex.exec(testText);
        if (m) list.push({ index: 1, match: m[0], groups: m.slice(1) });
      }
      setMatches(list);
    } catch {
      setMatches([]);
    }
  }, [pattern, flags, testText]);

  return (
    <div className="regex-tool">
      <div className="regex-tool__top-section">
        <div className="regex-tool__input-group">
          <label className="regex-tool__label">正则表达式</label>
          <div className="regex-tool__input-wrapper">
            <span className="regex-tool__slash">/</span>
            <input
              type="text"
              className="regex-tool__input"
              value={pattern}
              onChange={(e) => setPattern(e.target.value)}
              placeholder="输入正则表达式..."
              spellCheck={false}
            />
            <span className="regex-tool__slash regex-tool__slash--end">/</span>
            <div className="regex-tool__flags">
              {flagList.map((f) => (
                <span
                  key={f.key}
                  className={`regex-tool__flag ${flags[f.key] ? 'active' : ''}`}
                  onClick={() => toggleFlag(f.key)}
                  title={f.desc}
                >
                  {f.label}
                </span>
              ))}
            </div>
          </div>
        </div>

        <div className="regex-tool__quick-btns">
          {quickPatterns.map((p) => (
            <button
              key={p.label}
              className="regex-tool__quick-btn"
              onClick={() => applyPattern(p.pattern)}
            >
              {p.label}
            </button>
          ))}
        </div>

        <div className="regex-tool__input-group">
          <label className="regex-tool__label">测试文本</label>
          <textarea
            className="regex-tool__textarea"
            value={testText}
            onChange={(e) => setTestText(e.target.value)}
            placeholder="输入待测试的文本，例如: 我的邮箱是 test@example.com，手机号 13800138000，IP 192.168.1.1..."
            spellCheck={false}
          />
        </div>
      </div>

      <div className="regex-tool__results">
        <div className="regex-tool__results-header">
          <span className="regex-tool__results-title">匹配结果</span>
          <span className="regex-tool__results-count">{matches.length} 个匹配</span>
        </div>
        <div className="regex-tool__results-list">
          {matches.length === 0 ? (
            <div className="regex-tool__empty">
              <span className="regex-tool__empty-icon">//</span>
              <span>{pattern && testText ? '无匹配结果' : '输入正则表达式和测试文本'}</span>
            </div>
          ) : (
            matches.map((r) => (
              <div key={r.index} className="regex-tool__result-item">
                <span className="regex-tool__result-index">{r.index}</span>
                <span className="regex-tool__result-match">{r.match}</span>
                {r.groups.length > 0 && r.groups[0] !== undefined && (
                  <div className="regex-tool__result-groups">
                    捕获组: {r.groups.map((g, i) => <span key={i}>{g}</span>)}
                  </div>
                )}
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
}
