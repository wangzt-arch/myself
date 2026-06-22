import { useState, useRef, useMemo } from 'react';
import { generateRandomJson, templatesList } from './randomJson';
import './index.css';

export default function JsonTool() {
  const [input, setInput] = useState('');
  const [output, setOutput] = useState('');
  const [error, setError] = useState('');
  const [showTemplateModal, setShowTemplateModal] = useState(false);
  const [arraySize, setArraySize] = useState(5);
  const [copied, setCopied] = useState(false);
  const textareaRef = useRef(null);

  const inputStats = useMemo(() => ({
    chars: input.length,
    lines: input ? input.split('\n').length : 0,
  }), [input]);

  const outputStats = useMemo(() => ({
    chars: output.length,
    lines: output ? output.split('\n').length : 0,
  }), [output]);

  const formatJson = () => {
    if (!input.trim()) {
      setOutput('');
      setError('');
      return;
    }
    try {
      const parsed = JSON.parse(input);
      setOutput(JSON.stringify(parsed, null, 2));
      setError('');
    } catch (e) {
      setOutput('');
      const match = e.message.match(/position (\d+)/);
      if (match) {
        const pos = parseInt(match[1]);
        const lines = input.substring(0, pos).split('\n');
        setError(`第 ${lines.length} 行，第 ${lines[lines.length - 1].length + 1} 列: ${e.message}`);
      } else {
        setError(e.message);
      }
    }
  };

  const compressJson = () => {
    if (!input.trim()) return;
    try {
      setInput(JSON.stringify(JSON.parse(input)));
      setOutput('');
      setError('');
    } catch (e) {
      formatJson();
    }
  };

  const copyOutput = async () => {
    if (!output) return;
    try {
      await navigator.clipboard.writeText(output);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch (e) {
      // 降级方案
      const ta = document.createElement('textarea');
      ta.value = output;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      document.body.removeChild(ta);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    }
  };

  const clearAll = () => {
    setInput('');
    setOutput('');
    setError('');
    textareaRef.current?.focus();
  };

  const handleGenerate = (template) => {
    const randomData = generateRandomJson(template, arraySize);
    const jsonStr = JSON.stringify(randomData, null, 2);
    setInput(jsonStr);
    setOutput('');
    setError('');
    setShowTemplateModal(false);
    setTimeout(() => formatJson(), 0);
  };

  return (
    <div className="json-tool">
      <div className="json-tool__container">
        <div className="ot-card">
          <div className="ot-card__header">
            <span className="ot-card__title">输入 JSON</span>
            <div className="ot-card__actions">
              <button className="ot-btn" onClick={() => setShowTemplateModal(true)}>
                <span>✦</span>生成随机
              </button>
              <button className="ot-btn" onClick={clearAll}>清空</button>
              <button className="ot-btn ot-btn--primary" onClick={formatJson}>格式化</button>
              <button className="ot-btn" onClick={compressJson}>压缩</button>
            </div>
          </div>
          <textarea
            ref={textareaRef}
            className="ot-textarea"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Tab') {
                e.preventDefault();
                const start = e.target.selectionStart;
                const end = e.target.selectionEnd;
                const newValue = input.substring(0, start) + '  ' + input.substring(end);
                setInput(newValue);
                setTimeout(() => { e.target.selectionStart = e.target.selectionEnd = start + 2; }, 0);
              }
            }}
            placeholder='在此粘贴 JSON 数据，如: {"name":"LogicFlow","version":"1.0.0"}'
            spellCheck={false}
          />
          <div className={`json-tool__error ${error ? 'show' : ''}`}>
            <span className="json-tool__error-label">✕ 解析失败</span>
            {error}
          </div>
          <div className="json-tool__stats">
            <span>字符数<strong>{inputStats.chars}</strong></span>
            <span>行数<strong>{inputStats.lines}</strong></span>
          </div>
        </div>

        <div className="ot-card">
          <div className="ot-card__header">
            <span className="ot-card__title">格式化结果</span>
            <div className="ot-card__actions">
              <span className={`json-tool__copy-tip ${copied ? 'show' : ''}`}>✓ 已复制</span>
              <button className="ot-btn" onClick={copyOutput} disabled={!output}>复制</button>
            </div>
          </div>
          <textarea
            className="ot-textarea ot-textarea--output"
            value={output}
            readOnly
            placeholder="格式化后的 JSON 将显示在此..."
            spellCheck={false}
          />
          <div className="json-tool__stats">
            <span>字符数<strong>{outputStats.chars}</strong></span>
            <span>行数<strong>{outputStats.lines}</strong></span>
          </div>
        </div>
      </div>

      {showTemplateModal && (
        <div className="json-tool__modal-mask" onClick={() => setShowTemplateModal(false)}>
          <div className="json-tool__modal" onClick={(e) => e.stopPropagation()}>
            <div className="json-tool__modal-header">
              <span className="json-tool__modal-title">选择随机数据模板</span>
              <button className="json-tool__modal-close" onClick={() => setShowTemplateModal(false)}>✕</button>
            </div>
            <div className="json-tool__modal-body">
              <div className="json-tool__modal-size">
                <label>数组数量（仅数组模板生效）</label>
                <input
                  type="number"
                  min="1"
                  max="100"
                  value={arraySize}
                  onChange={(e) => setArraySize(Math.max(1, Math.min(100, Number(e.target.value) || 1)))}
                />
              </div>
              <div className="json-tool__modal-templates">
                {templatesList.map((t) => (
                  <button
                    key={t.id}
                    className="json-tool__template-item"
                    onClick={() => handleGenerate(t.id)}
                  >
                    <span className="json-tool__template-label">{t.label}</span>
                    <span className="json-tool__template-desc">{t.desc}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
